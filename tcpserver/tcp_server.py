#!/usr/bin/env python3
"""
P0 frame protocol test server.

Frame: AA 55 | CMD | LEN(LE16) | DATA | CRC16(LE16)
CRC:   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), over CMD+LEN+DATA

0x01 LINE   int32 x, int32 y, int32 z, uint32 feed_hz, uint16 seq
0x02 STOP
0x03 ESTOP
0x04 HOME
0x05 QUERY
0x81 STATUS uint8 state, uint8 queue_free, int32 x, int32 y, int32 z, uint16 last_seq
0x82 ACK    uint16 seq, uint8 code
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

SOF = b"\xAA\x55"
MAX_PAYLOAD = 64

CMD_LINE = 0x01
CMD_STOP = 0x02
CMD_ESTOP = 0x03
CMD_HOME = 0x04
CMD_QUERY = 0x05
CMD_STATUS = 0x81
CMD_ACK = 0x82

ACK_NAMES = {
    0: "OK",
    1: "QUEUE_FULL",
    2: "BAD_LENGTH",
    3: "BAD_PARAM",
    4: "UNSUPPORTED",
    5: "BUSY",
}

STATE_NAMES = {
    0: "IDLE",
    1: "MOVING",
    2: "ESTOP",
    3: "ERROR",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([cmd]) + struct.pack("<H", len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def pack_line(x: int, y: int, z: int, feed_hz: int, seq: int) -> bytes:
    payload = struct.pack("<iiiIH", x, y, z, feed_hz, seq & 0xFFFF)
    return pack_frame(CMD_LINE, payload)


class FrameStream:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while True:
            start = self.buf.find(SOF)
            if start < 0:
                if self.buf.endswith(b"\xAA"):
                    self.buf[:] = self.buf[-1:]
                else:
                    self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 5:
                break
            cmd = self.buf[2]
            length = struct.unpack_from("<H", self.buf, 3)[0]
            if length > MAX_PAYLOAD:
                del self.buf[0]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            rx_crc = struct.unpack_from("<H", self.buf, 5 + length)[0]
            if crc16_ccitt_false(body) == rx_crc:
                frames.append((cmd, bytes(self.buf[5:5 + length])))
                del self.buf[:total]
            else:
                del self.buf[0]
        return frames


def describe_frame(cmd: int, payload: bytes) -> str:
    if cmd == CMD_STATUS and len(payload) == 16:
        state, queue_free = payload[0], payload[1]
        x, y, z = struct.unpack_from("<iii", payload, 2)
        last_seq = struct.unpack_from("<H", payload, 14)[0]
        qf = "n/a" if queue_free == 0xFF else str(queue_free)
        return (f"STATUS state={STATE_NAMES.get(state, state)} queue_free={qf} "
                f"pos=({x},{y},{z}) last_seq={last_seq}")
    if cmd == CMD_ACK and len(payload) == 3:
        seq, code = struct.unpack_from("<HB", payload, 0)
        return f"ACK seq={seq} code={ACK_NAMES.get(code, code)}"
    return f"CMD=0x{cmd:02X} payload={payload.hex()}"


def print_frames(frames):
    for cmd, payload in frames:
        print(f"  RX {describe_frame(cmd, payload)}")


def send_frame(sock: socket.socket, frame: bytes, label: str):
    print(f"  TX {label}: {frame.hex()}")
    sock.sendall(frame)


def recv_for(sock: socket.socket, stream: FrameStream, seconds: float):
    received = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            data = sock.recv(4096)
        except socket.timeout:
            break
        if not data:
            print("  connection closed")
            break
        frames = stream.feed(data)
        print_frames(frames)
        received.extend(frames)
    return received


def wait_for_idle(sock: socket.socket, stream: FrameStream, expect_seq: int, timeout: float):
    print(f"  waiting for completion seq={expect_seq}")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        for cmd, payload in recv_for(sock, stream, min(0.2, max(0.01, remaining))):
            if cmd == CMD_STATUS and len(payload) == 16:
                state = payload[0]
                last_seq = struct.unpack_from("<H", payload, 14)[0]
                if state == 0 and last_seq == expect_seq:
                    return payload
            elif cmd == CMD_ACK and len(payload) == 3:
                seq, code = struct.unpack_from("<HB", payload, 0)
                if seq == expect_seq and code != 0:
                    print(f"  move rejected: {ACK_NAMES.get(code, code)}")
                    return None
    print("  completion timeout")
    return None


def reader_loop(sock: socket.socket, stream: FrameStream, stop_event: threading.Event):
    sock.settimeout(0.5)
    while not stop_event.is_set():
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data:
            break
        print_frames(stream.feed(data))
    stop_event.set()
    print("\n  connection closed")


def run_demo(sock: socket.socket, steps: int, feed: int):
    stream = FrameStream()
    timeout = max(3.0, abs(steps) / max(feed, 1) * 2.0 + 5.0)

    send_frame(sock, pack_line(steps, 0, 0, feed, 1), f"LINE {steps},0,0 @{feed} seq=1")
    if wait_for_idle(sock, stream, 1, timeout) is None:
        return

    send_frame(sock, pack_line(0, 0, 0, feed, 2), f"LINE 0,0,0 @{feed} seq=2")
    if wait_for_idle(sock, stream, 2, timeout) is None:
        return

    send_frame(sock, pack_frame(CMD_QUERY), "QUERY")
    recv_for(sock, stream, 1.0)


def build_shape(shape: str, segments: int):
    if shape == "diagonal":
        return [(800, 400, 0), (0, 0, 0)]
    if shape == "square":
        return [(800, 0, 0), (800, 800, 0), (0, 800, 0), (0, 0, 0)]
    if shape == "circle":
        points = []
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            points.append((round(400 + 400 * math.cos(angle)),
                           round(400 + 400 * math.sin(angle)), 0))
        points.append((0, 0, 0))
        return points
    return []


def run_shape(sock: socket.socket, shape: str, feed: int, segments: int):
    points = build_shape(shape, segments)
    if not points:
        print(f"unknown shape: {shape}")
        return

    stream = FrameStream()
    seq = 1
    current = (0, 0, 0)
    timeout = max(10.0, 800.0 / max(feed, 1) * 2.0 + 5.0)

    for target in points:
        x, y, z = target
        cx, cy, cz = current
        steps = max(abs(x - cx), abs(y - cy), abs(z - cz))
        print(f"  SEGMENT {seq}: {current} -> {target}, steps={steps}")
        send_frame(sock, pack_line(x, y, z, feed, seq), f"LINE {target} @{feed} seq={seq}")
        status = wait_for_idle(sock, stream, seq, timeout)
        if status is None:
            return
        pos = struct.unpack_from("<iii", status, 2)
        if pos != target:
            print(f"  pos mismatch: {pos} != {target}")
            return
        print(f"  pos OK: {pos}")
        current = target
        seq = (seq + 1) & 0xFFFF

    print(f"  shape {shape} complete")


def run_shape_stream(sock: socket.socket, shape: str, feed: int, segments: int):
    points = build_shape(shape, segments)
    if not points:
        print(f"unknown shape: {shape}")
        return

    stream = FrameStream()
    targets = {i + 1: target for i, target in enumerate(points)}

    current = (0, 0, 0)
    total_steps = 0
    for target in points:
        total_steps += max(abs(target[i] - current[i]) for i in range(3))
        current = target
    timeout = max(15.0, total_steps / max(feed, 1) * 2.0 + 10.0)

    for seq, target in targets.items():
        send_frame(sock, pack_line(*target, feed, seq), f"LINE {target} @{feed} seq={seq}")

    print(f"  waiting for {len(targets)} completions")
    done = {}
    acks = 0
    final_state = None
    deadline = time.monotonic() + timeout

    while len(done) < len(targets) and time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        for cmd, payload in recv_for(sock, stream, min(0.2, max(0.01, remaining))):
            if cmd == CMD_ACK and len(payload) == 3:
                seq, code = struct.unpack_from("<HB", payload, 0)
                if code != 0:
                    print(f"  move rejected seq={seq}: {ACK_NAMES.get(code, code)}")
                    return
                acks += 1
            elif cmd == CMD_STATUS and len(payload) == 16:
                state = payload[0]
                seq = struct.unpack_from("<H", payload, 14)[0]
                pos = struct.unpack_from("<iii", payload, 2)
                if seq in targets:
                    if pos != targets[seq]:
                        print(f"  pos mismatch seq={seq}: {pos} != {targets[seq]}")
                        return
                    if seq not in done:
                        done[seq] = pos
                        print(f"  pos OK: {pos}")
                    if seq == len(targets):
                        final_state = state

    if len(done) < len(targets):
        print(f"  stream timeout: {len(done)}/{len(targets)} completed")
        return
    if final_state != 0:
        print(f"  final state is not IDLE: {final_state}")
        return

    print(f"  stream complete: {len(done)} segments, {acks} ACKs")


def run_interactive(sock: socket.socket):
    stream = FrameStream()
    stop_event = threading.Event()
    thread = threading.Thread(target=reader_loop, args=(sock, stream, stop_event), daemon=True)
    thread.start()

    seq = 1
    print("commands: line X Y Z FEED | query | stop | estop | home | quit")
    while not stop_event.is_set():
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd == "quit":
                break
            if cmd == "line" and len(parts) == 5:
                frame = pack_line(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), seq)
                send_frame(sock, frame, f"LINE seq={seq}")
                seq = (seq + 1) & 0xFFFF
            elif cmd == "query":
                send_frame(sock, pack_frame(CMD_QUERY), "QUERY")
            elif cmd == "stop":
                send_frame(sock, pack_frame(CMD_STOP), "STOP")
            elif cmd == "estop":
                send_frame(sock, pack_frame(CMD_ESTOP), "ESTOP")
            elif cmd == "home":
                send_frame(sock, pack_frame(CMD_HOME), "HOME")
            else:
                print("  usage: line X Y Z FEED | query | stop | estop | home | quit")
        except ValueError as exc:
            print(f"  {exc}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--shape", choices=["diagonal", "square", "circle"])
    parser.add_argument("--stream", action="store_true")
    parser.add_argument("--segments", type=int, default=12)
    parser.add_argument("--steps", type=int, default=800)
    parser.add_argument("--feed", type=int, default=1000)
    args = parser.parse_args()

    if args.stream and not args.shape:
        print("--stream requires --shape")
        return 2

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"listening on {args.host}:{args.port}, waiting for ESP8266...")

    client, addr = server.accept()
    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"connected: {addr}")

    try:
        if args.shape:
            if args.stream:
                run_shape_stream(client, args.shape, args.feed, args.segments)
            else:
                run_shape(client, args.shape, args.feed, args.segments)
        elif args.demo:
            run_demo(client, args.steps, args.feed)
        else:
            run_interactive(client)
    finally:
        client.close()
        server.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
