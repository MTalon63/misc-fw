import argparse
import serial
import time
import sys
import os
import struct
import threading
import asyncio
import queue
from collections import defaultdict

# Max sizes prevent RAM exhaustion and natively exert TCP backpressure when full
HIGH_PRIORITY_QUEUE = queue.Queue()
NORMAL_PRIORITY_QUEUE = queue.Queue()
REAL_TIME_PRIORITY_QUEUE = queue.Queue()
mcu_fifo_level = 0
mcu_poly = None  # Current randomizer poly reported by the firmware ("poly toggled to X-bit")
write_lock = threading.Lock()   # Serialises all ser.write() calls across threads
status_queue = queue.Queue()    # Reader daemon delivers 'q' status responses here
ready_event = threading.Event() # Set by the reader daemon on the "RDY" token
seq_counters = defaultdict(int)

# MCU FIFO is known to be 128 KB (131072 bytes)
MAX_FIFO_SIZE = 131072
TARGET_FIFO_LEVEL = int(0.80 * MAX_FIFO_SIZE)

# Will be set after reading status
DUMMY_PAYLOAD_SIZE = 8  # default CCSDS payload size

# --- TX power knob ---------------------------------------------------------
# Safe operating bounds (see plans/power_knob_spec.md §C).
DAC_MIN, DAC_MAX = 0, 3      # TxDacGain, 3 dB steps (never >= 4: datasheet test mode)
MIX_MIN, MIX_MAX = 6, 14     # TxMixerGain, 2 dB steps; cap at the validated default 14
REF_DAC, REF_MIX = 3, 14     # level 100 reference (current hardware default)
SCALE_MIN, SCALE_MAX = 4000, 16000  # digital_scale fine trim (per charge.txt firmware)
DEFAULT_DAC, DEFAULT_MIX, DEFAULT_SCALE = 3, 14, 12000  # known firmware defaults


def power_to_analog(level):
    """Map a 0..100 power level to (dac_gain_idx, mixer_gain).

    100 -> (3, 14) = current default (max); 0 -> (0, 6) = quietest supported.
    Monotonic in attenuation; never exceeds (3, 14).
    """
    level = max(0, min(100, int(level)))
    req = (100 - level) * 0.25  # requested attenuation in dB from the reference
    best = None
    for dac in range(DAC_MIN, DAC_MAX + 1):
        dac_att = (REF_DAC - dac) * 3
        for mix in range(MIX_MIN, MIX_MAX + 1):
            att = dac_att + (REF_MIX - mix) * 2
            key = (abs(att - req), -dac, -mix)  # nearest; keep DAC high, then mixer high
            if best is None or key < best[0]:
                best = (key, dac, mix)
    return best[1], best[2]


def _status_int(status, key, default):
    """Parse an integer from a 'q' status value (e.g. 'DAC Gain: 3'), else default."""
    raw = status.get(key)
    if raw is None:
        return default
    try:
        return int(str(raw).split()[0])
    except (ValueError, IndexError):
        return default


def resolve_startup_gain(args, status):
    """Resolve the startup dac/mixer/scale for the pre-binary-mode G command.

    Precedence: explicit --dac/--mixer/--scale win over --power, which wins over
    the current device values (parsed from the 'q' status), which fall back to
    the known firmware defaults. Returns (dac, mixer, scale, use_scale, changed).
    """
    dac = _status_int(status, "DAC Gain", DEFAULT_DAC)
    mix = _status_int(status, "Mixer Gain", DEFAULT_MIX)
    scale = _status_int(status, "Digital Scale", DEFAULT_SCALE)

    if getattr(args, "power", None) is not None:
        lvl = max(0, min(100, int(args.power)))
        dac, mix = power_to_analog(lvl)

    if args.dac is not None:
        if not DAC_MIN <= int(args.dac) <= DAC_MAX:
            print(f"[WARN] --dac {args.dac} outside {DAC_MIN}..{DAC_MAX}; clamped.")
        dac = max(DAC_MIN, min(DAC_MAX, int(args.dac)))
    if args.mixer is not None:
        if not 0 <= int(args.mixer) <= 15:
            print(f"[WARN] --mixer {args.mixer} outside 0..15; clamped.")
        mix = max(0, min(15, int(args.mixer)))
    if args.scale is not None:
        if not SCALE_MIN <= int(args.scale) <= SCALE_MAX:
            print(f"[WARN] --scale {args.scale} outside {SCALE_MIN}..{SCALE_MAX}; clamped.")
        scale = max(SCALE_MIN, min(SCALE_MAX, int(args.scale)))

    changed = (
        getattr(args, "power", None) is not None
        or args.dac is not None
        or args.mixer is not None
        or args.scale is not None
    )
    return dac, mix, scale, args.scale is not None, changed


active_serial = [None]          # current pyserial handle, or None
streaming = threading.Event()   # set while the binary streaming loop is running
stream_paused = threading.Event()  # set to pause packet writes during a power change
stream_paused_ack = threading.Event()  # set by the streaming loop once it observes the pause

CONTROL_HELP = (
    "Live TX power control:\n"
    "  power <0..100>            (alias: p) set power level\n"
    "  g <dac 0-3> <mixer 0-15> [scale 4000-16000]  absolute setter\n"
    "  status                    (alias: q) print modulator status\n"
    "  help                      (alias: ?) this help\n"
    "  quit                      (alias: exit) not supported; use Ctrl-C"
)


def ctrl_send_line(text, need_status=False):
    """Send one control line to the MCU.

    In binary streaming mode a 300 ms write pause is created and the line is
    sent with a leading ESC (0x1B); the firmware honours it only after a quiet
    gap >= 250 ms (see power_knob_spec.md A5). No busy-spin; returns None when
    the port is down.
    """
    ser = active_serial[0]
    if ser is None or not getattr(ser, "is_open", False):
        print("[Ctrl] Serial port not available; command dropped.")
        return None
    if streaming.is_set():
        # Pause the streaming loop and WAIT for it to actually observe the pause
        # (i.e. finish any in-flight packet write) BEFORE starting the 250 ms
        # quiet-gap timer. Otherwise a packet written mid-race could land
        # < 250 ms before the ESC, so the firmware would treat the ESC as
        # payload bytes -> the command is dropped AND the stream is corrupted.
        stream_paused_ack.clear()
        stream_paused.set()
        stream_paused_ack.wait(timeout=1.0)
        time.sleep(0.30)  # > firmware BINARY_CTL_GAP_MS (250 ms)
        try:
            with write_lock:
                ser.write(("\x1b" + text + "\n").encode("utf-8"))
                ser.flush()
        except Exception as e:
            print(f"[Ctrl] send failed: {e}")
            return None
        finally:
            time.sleep(0.05)
            stream_paused.clear()
            stream_paused_ack.clear()
    else:
        try:
            with write_lock:
                ser.write((text + "\n").encode("utf-8"))
                ser.flush()
        except Exception as e:
            print(f"[Ctrl] send failed: {e}")
            return None
    if need_status:
        try:
            return status_queue.get(timeout=2.0)
        except queue.Empty:
            return b""
    return b""


def handle_control_line(line):
    parts = line.split()
    if not parts:
        return
    cmd = parts[0].lower()
    if cmd in ("help", "?"):
        print(CONTROL_HELP)
        return
    if cmd in ("quit", "exit"):
        print("[Ctrl] 'quit' is not supported; use Ctrl-C to stop the service.")
        return
    if cmd in ("status", "q"):
        if not ctrl_send_line("q", need_status=True):
            print("[Ctrl] status unavailable (port down).")
        else:
            print("[Ctrl] status requested (see q block above).")
        return
    if cmd in ("power", "p"):
        if len(parts) < 2:
            print("[Ctrl] usage: power <0..100>")
            return
        try:
            lvl = int(parts[1])
        except ValueError:
            print("[Ctrl] power level must be an integer 0..100")
            return
        dac, mix = power_to_analog(lvl)
        if ctrl_send_line(f"G {dac} {mix}") is not None:
            print(f"[Ctrl] power={max(0, min(100, lvl))} -> dac={dac} mixer={mix}")
        return
    if cmd == "g":
        if len(parts) < 3:
            print("[Ctrl] usage: g <dac 0-3> <mixer 0-15> [scale 4000-16000]")
            return
        try:
            dac = max(0, min(3, int(parts[1])))
            mix = max(0, min(15, int(parts[2])))
            scale = max(4000, min(16000, int(parts[3]))) if len(parts) > 3 else None
        except ValueError:
            print("[Ctrl] g expects integers: g <dac> <mixer> [scale]")
            return
        text = f"G {dac} {mix}" + (f" {scale}" if scale is not None else "")
        if ctrl_send_line(text) is not None:
            print(f"[Ctrl] set dac={dac} mixer={mix}"
                  + (f" scale={scale}" if scale is not None else ""))
        return
    print(f"[Ctrl] unknown command '{cmd}' (try 'help')")


def stdin_reader_daemon():
    """Blocking line reader on stdin. Daemon thread; no busy-spin."""
    while True:
        try:
            line = sys.stdin.readline()
        except (KeyboardInterrupt, EOFError):
            return
        except Exception as e:
            print(f"[Ctrl] stdin error: {e}")
            return
        if not line:  # EOF
            return
        line = line.strip()
        if line:
            try:
                handle_control_line(line)
            except Exception as e:
                print(f"[Ctrl] command error: {e}")


def get_status(ser):
    # Only the reader daemon may read from 'ser'; it routes complete 'q' status
    # responses to status_queue. This function just writes the request and waits.
    with write_lock:
        ser.write(b"q")
        ser.flush()
    try:
        response = status_queue.get(timeout=2.0)
    except queue.Empty:
        response = b""
    status = {}
    lines = response.decode("utf-8", errors="ignore").split("\n")
    for line in lines:
        line = line.strip()
        if ":" in line and not line.startswith("[MCU]"):
            key, val = line.split(":", 1)
            status[key.strip()] = val.strip()
    return status, response


def toggle_if_needed(ser, status, key, desired_state, toggle_cmd):
    current = status.get(key)
    if current is not None:
        is_on = current.startswith("ON")
        if (desired_state and not is_on) or (not desired_state and is_on):
            print(f"[Serial] Toggling {key} to {'ON' if desired_state else 'OFF'}...")
            with write_lock:
                ser.write(toggle_cmd.encode("utf-8"))
                ser.flush()
            time.sleep(0.1)
            return True
    return False


def toggle_poly_if_needed(ser, status, desired_poly):
    current = status.get("Randomizer")
    curr_poly = None
    if current is not None:
        if "Poly:" in current:
            try:
                curr_poly = int(current.split("Poly: ")[1].split("-bit")[0])
            except Exception:
                pass

    if curr_poly is None:
        # Firmware prints "Randomizer poly toggled to X-bit"; the reader daemon
        # records that into mcu_poly, so we do not read directly from the port.
        with write_lock:
            ser.write(b"N")
            ser.flush()
        time.sleep(0.1)
        curr_poly = mcu_poly

        if curr_poly is not None and curr_poly != desired_poly:
            with write_lock:
                ser.write(b"N")
                ser.flush()
            time.sleep(0.1)
        return True
    else:
        if curr_poly != desired_poly:
            print(f"[Serial] Toggling Randomizer Poly from {curr_poly}-bit to {desired_poly}-bit...")
            with write_lock:
                ser.write(b"N")
                ser.flush()
            time.sleep(0.1)
            return True
    return False


def parse_current_rrc_status(status):
    alpha = 0.35
    span = 8
    rrc_type = 1
    min_dac_rate = 20000000

    rrc_str = status.get("RRC Filter", "")
    if rrc_str and rrc_str.startswith("ON"):
        try:
            if "Type: RC" in rrc_str:
                rrc_type = 0
            elif "Type: RRC" in rrc_str:
                rrc_type = 1

            if "alpha=" in rrc_str:
                parts = rrc_str.split("alpha=")
                alpha = float(parts[1].split(",")[0].replace(")", "").strip())

            if "span=" in rrc_str:
                parts = rrc_str.split("span=")
                span = int(parts[1].split(",")[0].replace(")", "").strip())
        except Exception as e:
            print(f"[Serial] Warning: Failed to parse current RRC Filter status: {e}")

    return alpha, min_dac_rate, span, rrc_type


def generate_space_packet(apid, seq, payload):
    header = bytearray(6)
    header[0] = (apid >> 8) & 0x07
    header[1] = apid & 0xFF
    header[2] = 0xC0 | ((seq >> 8) & 0x3F)
    header[3] = seq & 0xFF
    pdl = len(payload) - 1
    header[4] = (pdl >> 8) & 0xFF
    header[5] = pdl & 0xFF
    return header + payload


def serial_worker(args):
    global mcu_fifo_level, TARGET_FIFO_LEVEL, DUMMY_PAYLOAD_SIZE
    port = args.port
    baud = args.baud
    ser_instance = [None]
    # Set by the port-lifecycle path to ask the reader to stop before the port
    # is closed/reopened, so a close cannot race an in-flight readline().
    stop_event = threading.Event()

    # A fresh stop Event is created for every reader_daemon incarnation. Setting
    # it (and joining the thread) before closing/reopening the port guarantees the
    # blocked readline() is abandoned before the underlying handles are torn down.
    # A list is used (matching ser_instance/reader_thread) so start_reader can
    # replace the Event for each new incarnation without rebinding this slot.
    reader_stop = [threading.Event()]
    reader_thread = [None]

    def reader_daemon(stop_event):
        global mcu_fifo_level, mcu_poly
        # Sole thread allowed to read from 'ser'. It routes complete 'q' status
        # responses to status_queue and records the poly/RDY tokens for the host.
        in_status = False
        status_buf = []
        error_logged = False
        while not stop_event.is_set():
            ser = ser_instance[0]
            if ser is None or not ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = ser.readline()
            except (serial.SerialException, OSError, TypeError) as e:
                # The reconnect path may close/reopen the port while we are
                # mid-read; on Windows that surfaces as TypeError from byref().
                # Never let a transient error kill the sole reader.
                if not error_logged:
                    print(f"[Serial] Reader transient error: {e}; waiting for port...")
                    error_logged = True
                time.sleep(0.1)
                continue
            error_logged = False
            if not line:
                continue
            text = line.decode("utf-8", errors="ignore").strip()
            if text == "=== Modulator Status ===":
                in_status = True
                status_buf = [text]
            elif in_status:
                status_buf.append(text)
                # "Core 1 Headroom" is the final line of the 'q' response.
                if text.startswith("Core 1 Headroom"):
                    status_queue.put(
                        "\n".join(status_buf).encode("utf-8")
                    )
                    in_status = False
                    status_buf = []
            elif "poly toggled to" in text:
                # Firmware: "Randomizer poly toggled to 8-bit"
                try:
                    mcu_poly = int(
                        text.split("poly toggled to ")[1].split("-bit")[0]
                    )
                except ValueError:
                    pass
            elif text == "RDY":
                ready_event.set()
            elif text.startswith("[MCU]"):
                if "FIFO:" in text:
                    try:
                        mcu_fifo_level = int(
                            text.split("FIFO: ")[1].split("/")[0]
                        )
                    except ValueError:
                        pass
                sys.stdout.write(
                    f"\r{text} | RT Q: {REAL_TIME_PRIORITY_QUEUE.qsize()} | High Q: {HIGH_PRIORITY_QUEUE.qsize()} | Normal Q: {NORMAL_PRIORITY_QUEUE.qsize()}    "
                )
                sys.stdout.flush()
            else:
                sys.stdout.write(f"\n{text}\n")
                sys.stdout.flush()

    def start_reader():
        t = threading.Thread(target=reader_daemon, daemon=True)
        t.start()
        return t

    reader_thread = start_reader()

    while True:
        try:
            print(f"[Serial] Connecting to {port} at {baud} baud...")
            ser = serial.Serial(port, baud, timeout=1.0, write_timeout=30.0)
            # Point the reader daemon (sole reader) at this port immediately.
            ser_instance[0] = ser
            active_serial[0] = ser

            # Send 'q' to see if it responds with status (read via the daemon).
            status, response = get_status(ser)
            if not status:
                print(
                    "[Serial] MCU is unresponsive (likely in stream mode). Waiting for the MCU to recover (no hardware watchdog is enabled); you may need to reset the device manually..."
                )
                time.sleep(15.5)

            print("[Serial] Restarting MCU...")
            with write_lock:
                ser.write(b"m")
                ser.flush()
            time.sleep(1.5)

            # Stop the reader before closing the port so the close/reopen cannot
            # race an in-flight readline() (which raises TypeError on Windows).
            stop_event.set()
            reader_thread.join(timeout=2.0)
            ser.close()
            ser_instance[0] = None
            active_serial[0] = None
            # Reap the reader if it was blocked mid-read when the port closed.
            reader_thread.join(timeout=2.0)
            time.sleep(0.5)

            print(f"[Serial] Reconnecting after MCU restart...")
            ser = serial.Serial(port, baud, timeout=10, write_timeout=30.0)
            ser_instance[0] = ser
            active_serial[0] = ser
            stop_event.clear()
            reader_thread = start_reader()

            status, response = get_status(ser)
            retry_count = 0
            while not status and retry_count < 10:
                time.sleep(0.5)
                status, response = get_status(ser)
                retry_count += 1

            if not status:
                raise serial.SerialException(
                    "MCU not responding to 'q' status command."
                )

            if any(
                a is not None
                for a in [
                    args.rate,
                    args.crate,
                    args.inter,
                    args.rs,
                    args.ldpc,
                    args.conv,
                    args.rand,
                    args.randpoly,
                    args.fecf,
                    args.rrc,
                    args.rrc_alpha,
                    args.rrc_span,
                    args.rrc_type,
                ]
            ):
                print("[Serial] Applying desired modulator settings...")

                if args.rate is not None:
                    print(f"[Serial] Setting symbol rate to {args.rate} Hz...")
                    with write_lock:
                        ser.write(f"r{args.rate}\n".encode("utf-8"))
                        ser.flush()
                    time.sleep(0.1)

                if args.crate is not None:
                    print(
                        f"[Serial] Setting convolutional rate to {['1/2','2/3','3/4','5/6','7/8'][args.crate]}..."
                    )
                    with write_lock:
                        ser.write(f"k{args.crate}\n".encode("utf-8"))
                        ser.flush()
                    time.sleep(0.1)

                if args.inter is not None:
                    print(
                        f"[WARN] Interleave is not supported by the current firmware "
                        f"(no 'l' handler); --inter {args.inter} is ignored."
                    )

                if args.rs is not None:
                    if toggle_if_needed(ser, status, "RS (255,223)", args.rs == 1, "y"):
                        status, _ = get_status(ser)
                if args.ldpc is not None:
                    if toggle_if_needed(
                        ser, status, "LDPC (8160,7136)", args.ldpc == 1, "L"
                    ):
                        status, _ = get_status(ser)
                if args.conv is not None:
                    if toggle_if_needed(
                        ser, status, "Convolutional", args.conv == 1, "c"
                    ):
                        status, _ = get_status(ser)
                if args.rand is not None:
                    if toggle_if_needed(ser, status, "Randomizer", args.rand == 1, "n"):
                        status, _ = get_status(ser)
                if args.randpoly is not None:
                    if toggle_poly_if_needed(ser, status, args.randpoly):
                        status, _ = get_status(ser)
                if args.fecf is not None:
                    if toggle_if_needed(
                        ser, status, "FECF (CRC-16)", args.fecf == 1, "e"
                    ):
                        status, _ = get_status(ser)

                needs_f_config = (
                    args.rrc_alpha is not None or
                    args.rrc_span is not None or
                    args.rrc_type is not None or
                    (args.rrc == 1 and not status.get("RRC Filter", "").startswith("ON"))
                )

                if needs_f_config:
                    curr_alpha, curr_min_dac, curr_span, curr_type = parse_current_rrc_status(status)
                    alpha = args.rrc_alpha if args.rrc_alpha is not None else curr_alpha
                    span = args.rrc_span if args.rrc_span is not None else curr_span
                    rrc_type = args.rrc_type if args.rrc_type is not None else curr_type

                    # The firmware RRC table always uses a fixed 5-symbol span and
                    # RRC only; warn when a value the firmware cannot honour is asked.
                    if args.rrc_span is not None and args.rrc_span != 5:
                        print(
                            f"[WARN] RRC span {args.rrc_span} is not supported "
                            f"by the current firmware (fixed 5-symbol span); it is ignored."
                        )
                    if args.rrc_type is not None and args.rrc_type != 1:
                        print(
                            f"[WARN] RRC type {args.rrc_type} is not supported by the "
                            f"current firmware (RRC only); it is ignored."
                        )
                    if args.rrc_alpha is not None and args.rrc_alpha not in (0.25, 0.35, 0.5):
                        print(
                            f"[WARN] RRC alpha {args.rrc_alpha} is not in {{0.25, 0.35, 0.5}}; "
                            f"coerced to 0.5."
                        )
                        alpha = 0.5

                    print(f"[Serial] Configuring RRC/RC filter: F 0 {span} {alpha} {rrc_type} (auto L)...")
                    with write_lock:
                        ser.write(f"F 0 {span} {alpha} {rrc_type}\n".encode("utf-8"))
                        ser.flush()
                    time.sleep(0.1)
                    status, _ = get_status(ser)

                if args.rrc == 0:
                    if toggle_if_needed(ser, status, "RRC Filter", False, "W"):
                        status, _ = get_status(ser)

                status, _ = get_status(ser)

            print("\n[Serial] --- Final Modulator Status ---")
            for k in [
                "Symbol Rate",
                "Frame Size",
                "RS Interleave",
                "RS (255,223)",
                "LDPC (8160,7136)",
                "FECF (CRC-16)",
                "Convolutional",
                "Randomizer",
                "RRC Filter",
                "Expected Payload",
            ]:
                print(f"[Serial]   {k}: {status.get(k, 'Unknown')}")

            try:
                sym_rate_str = status.get("Symbol Rate", f"{args.rate} Hz")
                symbol_rate = int(sym_rate_str.split()[0])

                frame_size_str = status.get("Frame Size", "1024 bytes")
                frame_size = int(frame_size_str.split()[0])

                conv_status = status.get("Convolutional", "OFF")
                
                # Frame Size (1024 bytes) already includes RS/LDPC parity and the Attached Sync Marker.
                base_bits_per_frame = float(frame_size * 8)
                symbols_per_frame = base_bits_per_frame

                # Account for Convolutional code expansion
                if "ON" in conv_status:
                    # Check status string first, fall back to args.crate if rate isn't in the string
                    crate_val = args.crate if args.crate is not None else 0
                    if "2/3" in conv_status or (crate_val == 1 and conv_status == "ON"):
                        symbols_per_frame = base_bits_per_frame * (3.0 / 2.0)
                    elif "3/4" in conv_status or (crate_val == 2 and conv_status == "ON"):
                        symbols_per_frame = base_bits_per_frame * (4.0 / 3.0)
                    elif "5/6" in conv_status or (crate_val == 3 and conv_status == "ON"):
                        symbols_per_frame = base_bits_per_frame * (6.0 / 5.0)
                    elif "7/8" in conv_status or (crate_val == 4 and conv_status == "ON"):
                        symbols_per_frame = base_bits_per_frame * (8.0 / 7.0)
                    else:
                        symbols_per_frame = base_bits_per_frame * 2.0

                # Account for OQPSK modulation (2 bits per symbol)
                symbols_per_frame /= 2.0

                frames_per_sec = symbol_rate / symbols_per_frame

                expected_payload_str = status.get("Expected Payload", "882 bytes")
                DUMMY_PAYLOAD_SIZE = int(expected_payload_str.split()[0])

                usable_bytes_per_sec = frames_per_sec * DUMMY_PAYLOAD_SIZE
                usable_kbps = (usable_bytes_per_sec * 8) / 1000
                
                TARGET_FIFO_LEVEL = int(usable_bytes_per_sec * 2)

                print(
                    f"[Serial]   Usable Bandwidth: {usable_kbps:.2f} kbps ({usable_bytes_per_sec/1024:.2f} KB/s)"
                )
                print(f"[Serial]   FIFO target: {TARGET_FIFO_LEVEL} bytes (~{TARGET_FIFO_LEVEL/MAX_FIFO_SIZE*100:.0f}% of {MAX_FIFO_SIZE} bytes)")
            except Exception:
                pass
            print("[Serial] ------------------------------\n")

            # Pre-binary-mode TX gain/scale: resolve dac/mixer/scale from
            # device/defaults -> --power -> explicit --dac/--mixer/--scale, then
            # send exactly ONE 'G' line iff any trigger knob flag was provided.
            dac, mix, scale, use_scale, gain_changed = resolve_startup_gain(args, status)
            if gain_changed:
                g_cmd = f"G {dac} {mix}" + (f" {scale}" if use_scale else "")
                print(f"[Serial] Setting TX gain: dac={dac}, mixer={mix}"
                      + (f", scale={scale}" if use_scale else ""))
                print(f"[Serial]   Sending: {g_cmd}")
                with write_lock:
                    ser.write(f"{g_cmd}\n".encode("utf-8"))
                    ser.flush()
                time.sleep(0.1)
                status, _ = get_status(ser)
                if status:
                    print(
                        f"[Serial]   TX DAC Gain: {status.get('DAC Gain', '?')}, "
                        f"Mixer Gain: {status.get('Mixer Gain', '?')}, "
                        f"Digital Scale: {status.get('Digital Scale', '?')}"
                    )
                else:
                    print("[Serial]   WARN: no status after G (MCU busy?); continuing.")

            with write_lock:
                ser.write(b"s")
                ser.flush()
            time.sleep(1.0)
            # Clear BEFORE requesting 'u' so a fast "RDY" from the reader daemon
            # cannot be lost between the write and the wait.
            ready_event.clear()
            with write_lock:
                ser.write(b"u")
                ser.flush()

            # The reader daemon sets ready_event on the dedicated "RDY" token the
            # firmware emits when entering binary mode.
            ready = ready_event.wait(timeout=3.0)

            if not ready:
                print(
                    "[Serial] Warning: Did not receive 'RDY' ready signal. Proceeding anyway."
                )
            else:
                print("[Serial] Modulator locked into Binary Mode. Stream ready!")

            ser_instance[0] = ser
            active_serial[0] = ser
            streaming.set()
            keep_alive_seq = 0
            last_user_packet_time = time.time()  # start the idle timer

            while True:
                if stream_paused.is_set():
                    stream_paused_ack.set()  # tell ctrl_send_line the stream has yielded
                    time.sleep(0.01)  # live power knob holds the stream briefly
                    continue
                apid = None
                payload = None
                priority_tag = "REALTIME"

                # Non-blocking checks for all priorities
                try:
                    apid, payload = REAL_TIME_PRIORITY_QUEUE.get_nowait()
                except queue.Empty:
                    priority_tag = "HIGH"
                    try:
                        apid, payload = HIGH_PRIORITY_QUEUE.get_nowait()
                    except queue.Empty:
                        priority_tag = "NORMAL"
                        try:
                            apid, payload = NORMAL_PRIORITY_QUEUE.get_nowait()
                        except queue.Empty:
                            # No user data – check if we need a keep‑alive dummy
                            if time.time() - last_user_packet_time >= 5.0:
                                # Send one full dummy frame to keep watchdog happy
                                dummy_payload = b'\x00' * max(0, DUMMY_PAYLOAD_SIZE - 6)
                                packet = generate_space_packet(
                                    2047,
                                    keep_alive_seq,
                                    dummy_payload,
                                )
                                keep_alive_seq = (keep_alive_seq + 1) & 0x3FFF

                                # Flow control before sending
                                deadline = time.time() + 5.0
                                while mcu_fifo_level > TARGET_FIFO_LEVEL and time.time() < deadline:
                                    time.sleep(0.001)
                                if mcu_fifo_level > TARGET_FIFO_LEVEL:
                                    print("[WARN] FIFO flow-control wait timed out; proceeding.")
                                with write_lock:
                                    ser.write(packet)
                                    ser.flush()
                                mcu_fifo_level += len(packet)
                                # Reset timer so we don't send another immediately
                                last_user_packet_time = time.time()
                            else:
                                # Idle – just yield CPU a bit
                                time.sleep(0.0001)
                            continue

                # If we get here, we have a user packet
                seq = seq_counters[apid]
                seq_counters[apid] = (seq + 1) & 0x3FFF

                packet = generate_space_packet(apid, seq, payload)

                # Flow control
                deadline = time.time() + 5.0
                while mcu_fifo_level > TARGET_FIFO_LEVEL and time.time() < deadline:
                    time.sleep(0.001)
                if mcu_fifo_level > TARGET_FIFO_LEVEL:
                    print("[WARN] FIFO flow-control wait timed out; proceeding.")

                with write_lock:
                    ser.write(packet)
                    ser.flush()
                mcu_fifo_level += len(packet)

                # Update the timestamp of the last user packet
                last_user_packet_time = time.time()

                if priority_tag == "REALTIME":
                    REAL_TIME_PRIORITY_QUEUE.task_done()
                elif priority_tag == "HIGH":
                    HIGH_PRIORITY_QUEUE.task_done()
                else:
                    NORMAL_PRIORITY_QUEUE.task_done()

        except ImportError as e:
            print(f"\n[Fatal Error] {e}")
            os._exit(1)
        except serial.SerialException as e:
            stop_event.set()
            reader_thread.join(timeout=2.0)
            ser_instance[0] = None
            active_serial[0] = None
            streaming.clear()
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
            print(f"\n[Serial] Connection lost: {e}")
            print("[Serial] Attempting to reconnect in 3 seconds...")
            time.sleep(3.0)
            stop_event.clear()
            reader_thread = start_reader()
        except Exception as e:
            stop_event.set()
            reader_thread.join(timeout=2.0)
            ser_instance[0] = None
            active_serial[0] = None
            streaming.clear()
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
            print(f"\n[Serial] Unexpected error: {e}")
            time.sleep(3.0)
            stop_event.clear()
            reader_thread = start_reader()


async def client_handler(reader, writer):
    addr = writer.get_extra_info("peername")
    print(f"[TCP] Client connected from {addr}")
    try:
        while True:
            try:
                header = await reader.readexactly(7)
            except asyncio.IncompleteReadError:
                break

            priority, apid, length = struct.unpack("!B H I", header)

            try:
                payload = await reader.readexactly(length)
            except asyncio.IncompleteReadError:
                break

            if priority == 2:
                await asyncio.to_thread(REAL_TIME_PRIORITY_QUEUE.put, (apid, payload))
            elif priority == 1:
                await asyncio.to_thread(HIGH_PRIORITY_QUEUE.put, (apid, payload))
            else:
                await asyncio.to_thread(NORMAL_PRIORITY_QUEUE.put, (apid, payload))

    except Exception as e:
        print(f"[TCP] Client {addr} error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"[TCP] Client disconnected: {addr}")


async def start_tcp_server(host, port):
    server = await asyncio.start_server(client_handler, host, port)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    print(f"[TCP] Service listening for incoming streams on {addrs}")

    async with server:
        await server.serve_forever()


def main():
    parser = argparse.ArgumentParser(
        description="Continuous BPSK USB/TCP Modulator Service"
    )
    parser.add_argument("port", help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    # Firmware uses 921600. The RP2350 CDC console ignores baud, but UART
    # transports would need it to match the firmware.
    parser.add_argument(
        "--baud", type=int, default=921600, help="Baud rate (default 921600)"
    )
    parser.add_argument(
        "--bind", default="0.0.0.0", help="TCP bind address (default 0.0.0.0)"
    )
    parser.add_argument(
        "--tcpport", type=int, default=8000, help="TCP listen port (default 8000)"
    )

    parser.add_argument(
        "--rate", type=int, default=1000000, help="Set symbol rate (Hz)"
    )
    parser.add_argument(
        "--crate",
        type=int,
        default=4,
        choices=[0, 1, 2, 3, 4],
        help="Set convolutional puncturing rate (0=1/2, 1=2/3, 2=3/4, 3=5/6, 4=7/8)",
    )
    parser.add_argument(
        "--inter",
        type=int,
        choices=[1, 2, 4, 5, 8],
        help="Set RS interleave depth",
    )
    parser.add_argument(
        "--rs",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Reed-Solomon",
    )
    parser.add_argument(
        "--ldpc",
        type=int,
        default=0,
        choices=[0, 1],
        help="0=Disable, 1=Enable LDPC",
    )
    parser.add_argument(
        "--conv",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Convolutional",
    )
    parser.add_argument(
        "--rand",
        type=int,
        default=1,
        choices=[0, 1],
        help="0=Disable, 1=Enable Randomizer",
    )
    parser.add_argument(
        "--randpoly",
        type=int,
        default=8,
        choices=[8, 17],
        help="CCSDS randomizer polynomial (8=8-bit, 17=17-bit)",
    )
    parser.add_argument(
        "--fecf", type=int, default=1, choices=[0, 1], help="0=Disable, 1=Enable FECF"
    )
    parser.add_argument(
        "--power",
        type=int,
        default=None,
        help="Initial TX power level 0..100 (100=max/default, 0=min). "
             "Omit to leave the hardware value unchanged.",
    )
    parser.add_argument(
        "--dac",
        type=int,
        default=None,
        help="TxDacGain index 0..3 (3 dB steps). Explicitly overrides any "
             "--power-derived dac. Omit to keep the current/device value.",
    )
    parser.add_argument(
        "--mixer",
        type=int,
        default=None,
        help="TxMixerGain 0..15 (2 dB steps). Explicitly overrides any "
             "--power-derived mixer. Omit to keep the current/device value.",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=None,
        help="digital_scale fine trim 4000..16000. Only sent when provided; "
             "otherwise the firmware leaves digital_scale unchanged.",
    )
    parser.add_argument(
        "--rrc",
        type=int,
        choices=[0, 1],
        help="0=Disable, 1=Enable RRC/RC pulse shaping filter",
    )
    parser.add_argument(
        "--rrc-alpha",
        type=float,
        help="RRC/RC filter roll-off factor (alpha, 0.05 to 0.95)",
    )
    parser.add_argument(
        "--rrc-span",
        type=int,
        help="RRC/RC filter span in symbols (2 to 12)",
    )
    parser.add_argument(
        "--rrc-type",
        type=int,
        choices=[0, 1],
        help="RRC/RC filter type (0=Raised Cosine, 1=Root Raised Cosine)",
    )
    parser.add_argument(
        "--qsize",
        type=int,
        default=500,
        help="Max items per priority queue (default 500, ~500KB at 1KB/item)",
    )

    args = parser.parse_args()

    global HIGH_PRIORITY_QUEUE, NORMAL_PRIORITY_QUEUE, REAL_TIME_PRIORITY_QUEUE
    HIGH_PRIORITY_QUEUE = queue.Queue(maxsize=args.qsize)
    NORMAL_PRIORITY_QUEUE = queue.Queue(maxsize=args.qsize)
    REAL_TIME_PRIORITY_QUEUE = queue.Queue(maxsize=args.qsize)

    serial_thread = threading.Thread(target=serial_worker, args=(args,), daemon=True)
    serial_thread.start()

    stdin_thread = threading.Thread(target=stdin_reader_daemon, daemon=True)
    stdin_thread.start()

    try:
        asyncio.run(start_tcp_server(args.bind, args.tcpport))
    except KeyboardInterrupt:
        print("\nShutting down Modulator Service...")
        sys.exit(0)


if __name__ == "__main__":
    main()