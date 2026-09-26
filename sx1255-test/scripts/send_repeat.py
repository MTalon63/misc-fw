#!/usr/bin/env python3
"""
send_repeat.py — Repeat-send selectable .bin payloads to the Modulator Service.

Like send_stream.py, but selected .bin files are split into fixed-size payload
chunks and sent in looping passes (forever by default) for an expo / demo.

Protocol (identical to send_stream.py line 229):
    header = struct.pack("!B H I", priority, apid, len(chunk)) + chunk
i.e.  [Priority:1B][APID:2B big-endian][Length:4B big-endian][Payload:Length].

The client sends RAW payload bytes only (NOT pre-framed CCSDS packets). The
service builds the 6-byte CCSDS Space Packet primary header and owns the
per-APID 14-bit sequence count, so the client must NOT manage the sequence.

Defaults: APID 10, 246-byte payloads, priority 0, host 127.0.0.1, port 8000.

Examples:
    python scripts/send_repeat.py gob.bin richy.bin
    python scripts/send_repeat.py gob.bin --loop-count 5
    python scripts/send_repeat.py --dir ./clips
    python scripts/send_repeat.py --dir ./clips --recursive --shuffle
"""

import argparse
import glob
import os
import random
import socket
import struct
import sys
import time

# Backoff between connect retries (seconds).
RETRY_DELAY = 2.0


def collect_files(files, directory, pattern, recursive):
    """Build the ordered, de-duplicated list of paths to send.

    Explicit positional files come first (their order preserved), then the
    --dir matches thrown an optional recursive glob. De-duplicate on the
    normalized absolute path, keeping first-seen order.
    """
    paths = list(files)
    if directory:
        if not os.path.isdir(directory):
            print(f"error: --dir is not a directory: {directory}", file=sys.stderr)
            sys.exit(2)
        pattern = os.path.join(directory, "**", pattern) if recursive else os.path.join(directory, pattern)
        found = sorted(
            glob.glob(pattern, recursive=True),
            key=lambda p: os.path.normcase(p),
        )
        paths += found

    seen = set()
    out = []
    for p in paths:
        key = os.path.normcase(os.path.abspath(p))
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def load_payloads(path, payload_size, pad):
    """Read a raw .bin file and split it into payload_size-byte chunks.

    The final short chunk is padded per `pad`:
      - "0x00": zero-padded up to payload_size (uniform packet size, default)
      - "0xff": padded with 0xFF
      - "none": emitted as-is (short final packet)
    An empty file yields no chunks (all zero-length input produces none).
    """
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        raise
    if not data:
        return []

    chunks = []
    for off in range(0, len(data), payload_size):
        chunk = data[off:off + payload_size]
        if len(chunk) < payload_size:
            if pad == "0x00":
                chunk = chunk + b"\x00" * (payload_size - len(chunk))
            elif pad == "0xff":
                chunk = chunk + b"\xff" * (payload_size - len(chunk))
            # pad == "none": leave the short chunk as-is
        chunks.append(chunk)
    return chunks


def connect(host, port):
    """Connect to the service, retrying with backoff until success or Ctrl+C."""
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            sock.connect((host, port))
            return sock
        except KeyboardInterrupt:
            raise
        except OSError as e:
            print(f"connect failed: {e}; retrying in {RETRY_DELAY:.0f}s (Ctrl+C to abort)", flush=True)
            try:
                time.sleep(RETRY_DELAY)
            except KeyboardInterrupt:
                raise


def send_chunk(sock, priority, apid, chunk):
    """Send one chunk using the send_stream.py wire framing."""
    sock.sendall(struct.pack("!B H I", priority, apid, len(chunk)) + chunk)


def send_with_reconnect(sock_ref, priority, apid, chunk, host, port):
    """Send a chunk, transparently reconnecting on mid-stream disconnect.

    On connection loss the current chunk (and therefore the current
    file/chunk position in the pass) is retried after a reconnect, so the
    pass resumes in place rather than restarting from the top. `sock_ref` is
    a one-element list holding the live socket so callers see the replacement.
    """
    while True:
        try:
            send_chunk(sock_ref[0], priority, apid, chunk)
            return
        except OSError as e:
            print(f"connection lost: {e}; reconnecting...", flush=True)
            try:
                sock_ref[0].close()
            except Exception:
                pass
            sock_ref[0] = connect(host, port)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Repeat-send selectable .bin files to the Modulator Service. "
            "Files are chunked into fixed-size payloads and sent in looping passes."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python scripts/send_repeat.py gob.bin richy.bin\n"
            "  python scripts/send_repeat.py gob.bin --loop-count 5\n"
            "  python scripts/send_repeat.py --dir ./clips\n"
            "  python scripts/send_repeat.py --dir ./clips --recursive --shuffle\n"
            "\n"
            "Note: packets carry a 246-byte payload by default. A trailing short\n"
            "chunk is zero-padded (--pad 0x00) so every packet is uniform; use\n"
            "--pad none when real data length must be preserved exactly (a short\n"
            "final packet is emitted, which the firmware tolerates via PDL)."
        ),
    )

    parser.add_argument("files", nargs="*", help="Explicit files to send, in order (extension ignored)")
    parser.add_argument("--dir", default=None, help="Directory whose *.bin files are sent")
    parser.add_argument("--recursive", action="store_true", help="With --dir, recurse into subdirs")
    parser.add_argument("--pattern", default="*.bin", help="Glob for --dir collection (default: *.bin)")

    parser.add_argument("--apid", type=int, default=10, help="APID passed to the service (0..0x7FF)")
    parser.add_argument("--payload-size", type=int, default=246, help="Payload bytes per packet (default: 246)")
    parser.add_argument("--pad", choices=["none", "0x00", "0xff"], default="0x00",
                        help="Fill for a final short chunk: 0x00 (uniform packets, default), 0xff, or none (short packet)")
    parser.add_argument("--priority", type=int, default=0, choices=[0, 1, 2],
                        help="Service queue: 0=NORMAL, 1=HIGH, 2=REALTIME (default: 0)")

    parser.add_argument("--host", default="127.0.0.1", help="Service host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Service TCP port (default: 8000)")

    parser.add_argument("--interval", type=float, default=0.0,
                        help="Seconds to sleep between packets (0.0 = rely on backpressure)")
    parser.add_argument("--rate", type=float, default=None,
                        help="Packets per second; sets interval=1/rate (mutually exclusive with --interval)")
    parser.add_argument("--once", action="store_true", help="Send selected files exactly once, then exit")
    parser.add_argument("--loop-count", type=int, default=None, help="Number of complete passes (default: loop forever)")
    parser.add_argument("--shuffle", action="store_true", help="Shuffle file order at the start of each pass")

    parser.add_argument("--verbose", action="store_true", help="Per-packet log lines")
    parser.add_argument("--stats-interval", type=float, default=1.0, help="Seconds between periodic stats lines (default: 1.0)")

    return parser


def validate(args):
    if not args.files and not args.dir:
        print("error: provide at least one file or --dir", file=sys.stderr)
        return False
    if not (0 <= args.apid <= 0x7FF):
        print("error: --apid must be in [0, 0x7FF]", file=sys.stderr)
        return False
    if not (1 <= args.payload_size <= 65535):
        print("error: --payload-size must be in [1, 65535]", file=sys.stderr)
        return False
    if args.interval and args.rate is not None:
        print("error: --interval and --rate are mutually exclusive", file=sys.stderr)
        return False
    if args.once and args.loop_count is not None:
        print("error: --once and --loop-count are mutually exclusive", file=sys.stderr)
        return False
    if args.loop_count is not None and args.loop_count < 1:
        print("error: --loop-count must be >= 1", file=sys.stderr)
        return False
    if args.rate is not None and args.rate <= 0:
        print("error: --rate must be > 0", file=sys.stderr)
        return False
    if args.stats_interval <= 0:
        print("error: --stats-interval must be > 0", file=sys.stderr)
        return False
    return True


def main():
    parser = build_parser()
    args = parser.parse_args()

    if not validate(args):
        sys.exit(2)

    # Pacer: only one of --interval / --rate is set at this point.
    interval = (1.0 / args.rate) if args.rate else args.interval

    files = collect_files(args.files, args.dir, args.pattern, args.recursive)
    if not files:
        print("error: no matching files to send", file=sys.stderr)
        sys.exit(2)

    # Preload: (path, [chunks]) for every sendable file.
    catalog = []
    for p in files:
        try:
            chunks = load_payloads(p, args.payload_size, args.pad)
        except OSError as e:
            print(f"warning: skip {p}: {e}", file=sys.stderr)
            continue
        if not chunks:
            print(f"warning: skip empty file: {p}", file=sys.stderr)
            continue
        print(f"loaded {p}: {len(chunks)} chunk(s) ({os.path.getsize(p)} bytes)", flush=True)
        catalog.append((p, chunks))
    if not catalog:
        print("error: no sendable files remain", file=sys.stderr)
        sys.exit(2)

    print(f"Connecting to Modulator Service at {args.host}:{args.port}...", flush=True)
    sock = connect(args.host, args.port)
    sock_ref = [sock]

    priority_label = {0: "NORMAL", 1: "HIGH", 2: "REALTIME"}[args.priority]
    print(f"Repeating {len(catalog)} file(s) | APID {args.apid} | payload {args.payload_size} B | "
          f"priority {priority_label} | pad {args.pad}", flush=True)

    sent = 0
    t0 = time.time()
    last_stats = t0
    passes = 0

    try:
        while True:
            order = list(catalog)
            if args.shuffle:
                random.shuffle(order)
            for path, chunks in order:
                for i, chunk in enumerate(chunks):
                    send_with_reconnect(sock_ref, args.priority, args.apid, chunk, args.host, args.port)
                    sent += 1

                    if args.verbose:
                        print(f"{path} chunk {i + 1}/{len(chunks)} len={len(chunk)} total={sent}", flush=True)

                    now = time.time()
                    if now - last_stats >= args.stats_interval:
                        elapsed = now - t0
                        kbps = (sent * args.payload_size * 8 / elapsed / 1000) if elapsed > 0 else 0.0
                        print(f"\rpasses={passes} packets={sent} apid={args.apid} {kbps:.1f} kbps",
                              end="", flush=True)
                        last_stats = now

                    if interval > 0:
                        time.sleep(interval)
            passes += 1
            if args.once and passes >= 1:
                break
            if args.loop_count is not None and passes >= args.loop_count:
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.", flush=True)
    finally:
        try:
            sock_ref[0].close()
        except Exception:
            pass
        print(f"\nDone. passes={passes} packets={sent}", flush=True)


if __name__ == "__main__":
    main()