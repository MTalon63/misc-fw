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

    def reader_daemon():
        global mcu_fifo_level, mcu_poly
        # Sole thread allowed to read from 'ser'. It routes complete 'q' status
        # responses to status_queue and records the poly/RDY tokens for the host.
        in_status = False
        status_buf = []
        while True:
            ser = ser_instance[0]
            if ser and ser.is_open:
                try:
                    line = ser.readline()
                    if line:
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
                except serial.SerialException:
                    pass
            else:
                time.sleep(0.1)

    threading.Thread(target=reader_daemon, daemon=True).start()

    while True:
        try:
            print(f"[Serial] Connecting to {port} at {baud} baud...")
            ser = serial.Serial(port, baud, timeout=1.0, write_timeout=30.0)
            # Point the reader daemon (sole reader) at this port immediately.
            ser_instance[0] = ser

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
            ser.close()
            ser_instance[0] = None
            time.sleep(0.5)

            print(f"[Serial] Reconnecting after MCU restart...")
            ser = serial.Serial(port, baud, timeout=10, write_timeout=30.0)
            ser_instance[0] = ser

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
            keep_alive_seq = 0
            last_user_packet_time = time.time()  # start the idle timer

            while True:
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
            ser_instance[0] = None
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
            print(f"\n[Serial] Connection lost: {e}")
            print("[Serial] Attempting to reconnect in 3 seconds...")
            time.sleep(3.0)
        except Exception as e:
            ser_instance[0] = None
            if "ser" in locals() and getattr(ser, "is_open", False):
                ser.close()
            print(f"\n[Serial] Unexpected error: {e}")
            time.sleep(3.0)


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

    try:
        asyncio.run(start_tcp_server(args.bind, args.tcpport))
    except KeyboardInterrupt:
        print("\nShutting down Modulator Service...")
        sys.exit(0)


if __name__ == "__main__":
    main()