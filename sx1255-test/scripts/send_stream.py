import argparse
import socket
import sys
import struct
import os
import time

def read_nal_units(stream, chunk_size=4096):
    buf = bytearray()
    while True:
        chunk = stream.read(chunk_size)

        if not chunk:
            if buf:
                max_fragment_size = 1400
                for offset in range(0, len(buf), max_fragment_size):
                    yield bytes(buf[offset:offset+max_fragment_size])
            break
        buf.extend(chunk)

        while True:
            if len(buf) < 4:
                break

            if buf[0:4] == b'\x00\x00\x00\x01':
                start_offset = 4
            elif buf[0:3] == b'\x00\x00\x01':
                start_offset = 3
            else:
                idx = buf.find(b'\x00\x00\x01')
                if idx == -1:
                    if len(buf) > 3:
                        del buf[:-3]
                    break
                else:
                    if idx > 0 and buf[idx-1] == 0:
                        idx -= 1
                    del buf[:idx]
                    continue

            idx = buf.find(b'\x00\x00\x01', start_offset)
            if idx == -1:
                break

            if idx > 0 and buf[idx-1] == 0:
                nal_len = idx - 1
            else:
                nal_len = idx

            nal = buf[:nal_len]
            max_fragment_size = 1400
            for offset in range(0, len(nal), max_fragment_size):
                yield bytes(nal[offset:offset+max_fragment_size])
            del buf[:nal_len]


def read_exact(stream, size):
    buf = bytearray()
    while len(buf) < size:
        needed = size - len(buf)
        chunk = stream.read(needed)
        if not chunk:
            break
        buf.extend(chunk)
    return bytes(buf)


def read_mpegts(stream, packets_per_chunk=4):
    buf = bytearray()
    chunk_size = 188 * packets_per_chunk
    
    try:
        is_stdin = (hasattr(stream, "fileno") and stream.fileno() == 0)
    except Exception:
        is_stdin = False

    while True:
        needed = chunk_size - len(buf)
        if needed > 0:
            chunk = stream.read(needed)
                
            if not chunk:
                if len(buf) > 0:
                    yield bytes(buf)
                break
            buf.extend(chunk)

        while len(buf) >= chunk_size:
            if buf[0] != 0x47:
                idx = buf.find(b'\x47')
                if idx == -1:
                    del buf[:]
                    break
                else:
                    del buf[:idx]
                    continue
            
            yield bytes(buf[:chunk_size])
            del buf[:chunk_size]

def main():
    parser = argparse.ArgumentParser(
        description="Client to push files/streams to the Modulator Service"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Service host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Service port (default: 8000)"
    )
    parser.add_argument(
        "--apid", type=int, default=1, help="APID for these packets (default: 1)"
    )
    parser.add_argument(
        "--high-priority", action="store_true", help="Route to High Priority queue"
    )
    parser.add_argument(
        "--realtime", action="store_true", help="Route to Real-Time Priority queue"
    )
    parser.add_argument(
        "--size",
        type=int,
        default=1000,
        help="Payload chunk size per packet (default: 1000)",
    )
    parser.add_argument(
        "--h264",
        action="store_true",
        help="Parse input stream as raw H.264 Annex-B NAL units",
    )
    parser.add_argument(
        "--mpegts",
        action="store_true",
        help="Parse input stream as MPEG-TS (188 bytes) and group into 4 packets per payload (752 bytes)",
    )
    parser.add_argument("file", help="File to send (use '-' for live stdin streaming)")
    args = parser.parse_args()
    
    if args.mpegts:
        if args.apid == 1:
            args.apid = 20 # Default to 20 for MPEG-TS
        args.size = 752
        args.realtime = True

    if args.realtime:
        priority = 2
    elif args.high_priority:
        priority = 1
    else:
        priority = 0

    print(f"Connecting to Modulator Service at {args.host}:{args.port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception as e:
        print(f"Failed to connect to service: {e}")
        sys.exit(1)

    f = sys.stdin.buffer if args.file == "-" else open(args.file, "rb")
    total_size = os.path.getsize(args.file) if args.file != "-" else None

    priority_label = {2: 'REALTIME', 1: 'HIGH', 0: 'NORMAL'}[priority]
    print(
        f"Streaming data with APID {args.apid} | Priority: {priority_label}..."
    )
    bytes_sent = 0
    start_time = time.time()
    last_update = start_time - 0.5
    last_bytes = 0

    try:
        if args.h264:
            for chunk in read_nal_units(f):
                # Protocol: [Priority: 1B] [APID: 2B] [Length: 4B] [Payload...]
                header = struct.pack("!B H I", priority, args.apid, len(chunk))
                sock.sendall(header + chunk)
                bytes_sent += len(chunk)

                now = time.time()
                if now - last_update >= 0.5:
                    speed_kbps = (
                        ((bytes_sent - last_bytes) * 8) / (now - last_update)
                    ) / 1000
                    last_bytes = bytes_sent
                    last_update = now
                    if total_size:
                        percent = (bytes_sent / total_size) * 100
                        sys.stdout.write(
                            f"\r[{percent:.1f}%] Sent {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    else:
                        sys.stdout.write(
                            f"\rStreaming... | Sent {bytes_sent/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    sys.stdout.flush()
        elif args.mpegts:
            for chunk in read_mpegts(f, packets_per_chunk=4):
                # Protocol: [Priority: 1B] [APID: 2B] [Length: 4B] [Payload...]
                header = struct.pack("!B H I", priority, args.apid, len(chunk))
                sock.sendall(header + chunk)
                bytes_sent += len(chunk)

                now = time.time()
                if now - last_update >= 0.5:
                    speed_kbps = (
                        ((bytes_sent - last_bytes) * 8) / (now - last_update)
                    ) / 1000
                    last_bytes = bytes_sent
                    last_update = now
                    if total_size:
                        percent = (bytes_sent / total_size) * 100
                        sys.stdout.write(
                            f"\r[{percent:.1f}%] Sent {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    else:
                        sys.stdout.write(
                            f"\rStreaming MPEG-TS... | Sent {bytes_sent/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    sys.stdout.flush()
        else:
            while True:
                chunk = read_exact(f, args.size)
                if not chunk:
                    break

                # Protocol: [Priority: 1B] [APID: 2B] [Length: 4B] [Payload...]
                header = struct.pack("!B H I", priority, args.apid, len(chunk))
                sock.sendall(header + chunk)
                bytes_sent += len(chunk)

                now = time.time()
                if now - last_update >= 0.5:
                    speed_kbps = (
                        ((bytes_sent - last_bytes) * 8) / (now - last_update)
                    ) / 1000
                    last_bytes = bytes_sent
                    last_update = now
                    if total_size:
                        percent = (bytes_sent / total_size) * 100
                        sys.stdout.write(
                            f"\r[{percent:.1f}%] Sent {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    else:
                        sys.stdout.write(
                            f"\rStreaming... | Sent {bytes_sent/1024:.1f} KB | {speed_kbps:.1f} kbps   "
                        )
                    sys.stdout.flush()

        # Print final progress update
        now = time.time()
        duration = now - start_time if now > start_time else 0.001
        speed_kbps = (bytes_sent * 8 / duration) / 1000
        if total_size:
            percent = (bytes_sent / total_size) * 100
            sys.stdout.write(
                f"\r[{percent:.1f}%] Sent {bytes_sent/1024:.1f}/{total_size/1024:.1f} KB | {speed_kbps:.1f} kbps   "
            )
        else:
            sys.stdout.write(
                f"\rStreaming complete | Sent {bytes_sent/1024:.1f} KB | {speed_kbps:.1f} kbps   "
            )
        sys.stdout.flush()
    except Exception as e:
        print(f"\nError sending data: {e}")
    finally:
        if args.file != "-":
            f.close()
        sock.close()
        print("\nTransmission complete. Socket closed.")


if __name__ == "__main__":
    main()