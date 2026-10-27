#!/usr/bin/env python3
"""AT32F456 execution firmware: SocketCAN status / direct motion / STOP.

Only Python's standard library is required. Power-cycle the chassis board before
each spin run: Reference sequence history is deliberately retained by firmware.
reset explicitly sends FAULT_RESET. This tool never flashes firmware or sends BMS commands.
"""

import argparse
import math
import signal
import socket
import struct
import sys
import time


FD_FRAME = struct.Struct("=IBB2x64s")
FAULTS = {1: "BUMPER_LATCH", 2: "ESTOP_COMMAND", 4: "RESERVED_0x04", 8: "CAN_LINK",
          16: "IMU_INVALID", 32: "RESERVED_0x20", 64: "ADC_INVALID"}


class Chassis:
    def __init__(self, interface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self.sock.bind((interface,))
            self.sock.settimeout(0.02)
        except BaseException:
            self.sock.close()
            raise
        self.status = None
        self.status_time = 0.0
        self.velocity = 0
        self.odometry_time = 0.0
        self.position = (0, 0)
        self.sequence = 0

    def send(self, can_id, payload):
        # CAN FD + BRS. Even the 8-byte control payload must use an FD frame.
        self.sock.send(FD_FRAME.pack(can_id, len(payload), 1, payload))

    def receive(self):
        try:
            packet = self.sock.recv(FD_FRAME.size)
        except socket.timeout:
            return
        if len(packet) != FD_FRAME.size:
            return
        can_id, size, flags, data = FD_FRAME.unpack(packet)
        if not flags & 1:
            return
        if can_id == 0x181:
            if size != 24:
                raise RuntimeError("0x181 length is not 24: firmware is not this V0.3 protocol")
            self.status = {
                "state": data[0], "fault": data[1], "bumper": data[2],
                "flags": data[3], "sequence": struct.unpack_from("<H", data, 4)[0],
                "result": data[6],
                "pwm": struct.unpack_from("<hh", data, 12),
                "power": struct.unpack_from("<HHH", data, 16),
            }
            self.status_time = time.monotonic()
        elif can_id == 0x381 and size == 20:
            self.position = struct.unpack_from("<ii", data, 0)
            self.velocity = struct.unpack_from("<h", data, 12)[0]
            self.odometry_time = time.monotonic()

    def wait(self, predicate, seconds=2.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.receive()
            if self.status is not None and predicate(self.status):
                return self.status
        raise RuntimeError("timed out waiting for chassis status/command acknowledgement")

    def show(self):
        s = self.status
        faults = "|".join(name for bit, name in FAULTS.items() if s["fault"] & bit)
        print(f"state={s['state']} fault=0x{s['fault']:02x}({faults or 'NONE'}) "
              f"bumper=0x{s['bumper']:02x} flags=0x{s['flags']:02x} "
              f"cmd={s['sequence']} result={s['result']} "
              f"pwm={s['pwm']} bus_mV,left_mA,right_mA={s['power']} "
              f"v_mm_s={self.velocity}", flush=True)

    def control(self, code):
        self.sequence = (self.sequence + 1) & 0xffff
        self.send(0x201, struct.pack("<BBHI", code, 0, self.sequence, 0))
        result = self.wait(lambda s: s["sequence"] == self.sequence)
        if result["result"] != 0:
            self.show()
            raise RuntimeError(f"command {code} was rejected; resolve the cause before retrying")
        return result

    def spin(self, distance_test=False, reference_start=0, feedback_timeout=0.35, ttl_ms=120):
        s = self.status
        if s["state"] not in (0, 1):
            raise RuntimeError("need a stationary board; release an explicit ESTOP with reset first")
        speeds = (50, 100, 150, 200, 250, 300, 250, 200, 150, 100, 50)
        if distance_test:
            self.wait(lambda _: time.monotonic() - self.odometry_time < 0.2)
        last_position = self.position
        distance_mm = 0.0
        print("Raised wheels only. " + ("11 metres of odometry, speed 50..300..50 mm/s."
              if distance_test else "50 mm/s forward reference for 3 seconds.") + " Ctrl+C stops.")
        try:
            # New execution firmware accepts 0x401 directly; no ENABLE transaction.
            self.velocity = 0
            if not distance_test:
                self.odometry_time = 0.0
            start = time.monotonic()
            last_sent_speed = None
            next_print = start
            reference_sequence = reference_start
            running_seen = False
            movement_seen = False
            while (distance_mm < 11000 if distance_test else time.monotonic() - start < 3.0):
                now = time.monotonic()
                if now - start >= 180:
                    raise RuntimeError("180-second test limit reached")
                speed = speeds[min(int(distance_mm // 1000), len(speeds) - 1)] if distance_test else 50
                if speed != last_sent_speed:
                    reference_sequence = (reference_sequence + 1) & 0xffff
                    payload = struct.pack("<HHhhHHI", reference_sequence, ttl_ms,
                                          speed, 0, 100, 500, 0)
                    self.send(0x401, payload)
                    last_sent_speed = speed
                self.receive()
                if distance_test:
                    distance_mm += math.hypot(self.position[0] - last_position[0],
                                              self.position[1] - last_position[1])
                    last_position = self.position
                    if time.monotonic() - self.odometry_time > feedback_timeout:
                        raise RuntimeError("odometry feedback lost")
                    if now - start > 2.0 and distance_mm < 10:
                        raise RuntimeError("no wheel-travel feedback after 2 seconds")
                s = self.status
                if time.monotonic() - self.status_time > feedback_timeout:
                    raise RuntimeError("chassis feedback lost")
                if s["state"] == 3:
                    self.show()
                    raise RuntimeError("upper-controller ESTOP received")
                if running_seen and s["state"] == 0:
                    raise RuntimeError("upper-controller STOP/DISABLE received")
                if s["state"] == 2:
                    running_seen = True
                if self.odometry_time >= start and abs(self.velocity) >= 5:
                    movement_seen = True
                if now >= next_print:
                    self.show()
                    if distance_test:
                        print(f"TRAVELLED_MM={distance_mm:.1f} TARGET_MM_S={speed} ELAPSED_S={now-start:.1f}", flush=True)
                    next_print = now + 0.5
            if not running_seen or not movement_seen:
                raise RuntimeError("running/movement feedback missing; verify firmware and power-cycle before retry")
        finally:
            # STOP clears the target. This sender sends no further Reference afterward.
            self.control(3)
            self.wait(lambda s: s["state"] in (0, 3) and s["pwm"] == (0, 0))
            self.show()
            print("STOP acknowledged; disabled/fault state and zero PWM confirmed.")
        if distance_test:
            print(f"PASS ODOMETRY_DISTANCE_MM={distance_mm:.1f} ELAPSED_S={time.monotonic()-start:.1f}")


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", choices=("status", "spin", "distance", "stop", "reset"), default="status")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--reference-start", type=int, default=0, help="last used Reference sequence, 0..65535")
    args = parser.parse_args()
    if not 0 <= args.reference_start <= 65535:
        parser.error("reference-start must be 0..65535")
    signal.signal(signal.SIGTERM, interrupt)
    board = Chassis(args.interface)
    try:
        board.wait(lambda _: True)
        board.sequence = board.status["sequence"]
        board.show()
        if args.mode in ("spin", "distance"):
            board.spin(args.mode == "distance", reference_start=args.reference_start)
        elif args.mode == "reset":
            board.control(5)
            board.show()
        elif args.mode == "stop":
            board.control(3)
            board.wait(lambda s: s["state"] in (0, 3) and s["pwm"] == (0, 0))
            board.show()
    finally:
        board.sock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted. Inspect STOP acknowledgement above; use physical power-off if unconfirmed.", file=sys.stderr)
        sys.exit(130)
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
