"""
PN532 HSU 串口写卡工具。

第一版针对 MIFARE Classic 1K，使用扇区1的数据块4和5保存同一份AGV记录。
工具只访问普通数据块，不修改UID、制造商块或扇区控制块。
"""

from __future__ import annotations

import argparse
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Optional

import serial
from serial.tools import list_ports


PN532_PREAMBLE = b"\x00\x00\xff"
PN532_HOST_TO_PN532 = 0xD4
PN532_PN532_TO_HOST = 0xD5
PN532_ACK = b"\x00\x00\xff\x00\xff\x00"
PN532_COMMAND_GET_FIRMWARE = 0x02
PN532_COMMAND_SAM_CONFIGURATION = 0x14
PN532_COMMAND_IN_LIST_PASSIVE_TARGET = 0x4A
PN532_COMMAND_IN_DATA_EXCHANGE = 0x40
MIFARE_AUTHENTICATE_A = 0x60
MIFARE_READ = 0x30
MIFARE_WRITE = 0xA0
MIFARE_CLASSIC_1K_SAK = 0x08
MIFARE_CLASSIC_4K_SAK = 0x18
MIFARE_CLASSIC_MINI_SAK = 0x09
DATA_BLOCK_PRIMARY = 4
DATA_BLOCK_BACKUP = 5
RECORD_MAGIC = b"AGV1"
RECORD_VERSION = 1
RECORD_SIZE = 16


class RfidError(RuntimeError):
    """RFID通信或数据校验错误。"""


def crc16_ccitt(data: bytes) -> int:
    """计算CRC-16/CCITT-FALSE，初值0xFFFF。"""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class AgvRecord:
    """写入MIFARE数据块的AGV位置记录。"""

    tag_type: int
    station_id: int
    map_version: int
    flags: int
    sequence: int

    def pack(self) -> bytes:
        body = struct.pack(
            "<4sBBHHHH",
            RECORD_MAGIC,
            RECORD_VERSION,
            self.tag_type,
            self.station_id,
            self.map_version,
            self.flags,
            self.sequence,
        )
        return body + struct.pack("<H", crc16_ccitt(body))

    @classmethod
    def unpack(cls, data: bytes) -> "AgvRecord":
        if len(data) != RECORD_SIZE:
            raise RfidError(f"卡片记录长度错误: {len(data)}")
        body = data[:14]
        stored_crc = struct.unpack_from("<H", data, 14)[0]
        if crc16_ccitt(body) != stored_crc:
            raise RfidError("卡片记录CRC校验失败")
        magic, version, tag_type, station_id, map_version, flags, sequence = (
            struct.unpack("<4sBBHHHH", body)
        )
        if magic != RECORD_MAGIC or version != RECORD_VERSION:
            raise RfidError("卡片记录标识或版本不匹配")
        return cls(tag_type, station_id, map_version, flags, sequence)


@dataclass(frozen=True)
class PassiveTarget:
    """PN532返回的ISO14443-A标签信息。"""

    target_number: int
    atqa: int
    sak: int
    uid: bytes

    @property
    def uid_text(self) -> str:
        return self.uid.hex(" ").upper()

    @property
    def is_mifare_classic(self) -> bool:
        return self.sak in {
            MIFARE_CLASSIC_MINI_SAK,
            MIFARE_CLASSIC_1K_SAK,
            MIFARE_CLASSIC_4K_SAK,
        }


class Pn532Hsu:
    """PN532 HSU协议封装。"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = serial.Serial(port, baudrate, timeout=0.1)

    def close(self) -> None:
        self.serial.close()

    def _read_exact(self, size: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        result = bytearray()
        while len(result) < size and time.monotonic() < deadline:
            chunk = self.serial.read(size - len(result))
            if chunk:
                result.extend(chunk)
        if len(result) != size:
            raise RfidError(f"串口接收超时，需要{size}字节，实际{len(result)}字节")
        return bytes(result)

    def _read_frame(self, timeout: float) -> Optional[bytes]:
        """读取一个普通PN532帧，返回None表示ACK帧。"""

        deadline = time.monotonic() + timeout
        state = 0
        while time.monotonic() < deadline:
            byte = self.serial.read(1)
            if not byte:
                continue
            value = byte[0]
            if state == 0:
                state = 1 if value == 0x00 else 0
            elif state == 1:
                state = 2 if value == 0x00 else 0
            else:
                if value == 0xFF:
                    break
                state = 1 if value == 0x00 else 0
        else:
            raise RfidError("等待PN532帧头超时")

        length = self._read_exact(1, timeout)[0]
        length_checksum = self._read_exact(1, timeout)[0]
        if length == 0xFF and length_checksum == 0x00:
            self._read_exact(1, timeout)
            raise RfidError("PN532返回NACK")
        if ((length + length_checksum) & 0xFF) != 0:
            raise RfidError("PN532帧长度校验失败")
        if length == 0:
            self._read_exact(1, timeout)
            return None
        if length > 64:
            raise RfidError(f"PN532帧过长: {length}")

        data = self._read_exact(length, timeout)
        data_checksum = self._read_exact(1, timeout)[0]
        self._read_exact(1, timeout)
        if ((sum(data) + data_checksum) & 0xFF) != 0:
            raise RfidError("PN532帧数据校验失败")
        return data

    @staticmethod
    def _build_frame(command: int, parameters: bytes) -> bytes:
        data = bytes([PN532_HOST_TO_PN532, command]) + parameters
        length = len(data)
        if length > 254:
            raise RfidError("PN532命令帧过长")
        return (
            PN532_PREAMBLE
            + bytes([length, (-length) & 0xFF])
            + data
            + bytes([(-sum(data)) & 0xFF, 0x00])
        )

    def wakeup(self) -> None:
        self.serial.reset_input_buffer()
        self.serial.write(b"\x55\x55\x00\x00\x00")
        self.serial.flush()
        time.sleep(0.05)
        self.serial.reset_input_buffer()

    def command(self, command: int, parameters: bytes = b"", timeout: float = 1.0) -> bytes:
        self.serial.reset_input_buffer()
        self.serial.write(self._build_frame(command, parameters))
        self.serial.flush()
        ack = self._read_frame(timeout)
        if ack is not None:
            raise RfidError("PN532未返回ACK")
        response = self._read_frame(timeout)
        if response is None or len(response) < 2:
            raise RfidError("PN532响应为空")
        if response[0] != PN532_PN532_TO_HOST:
            raise RfidError(f"PN532响应方向错误: 0x{response[0]:02X}")
        if response[1] != (command + 1) & 0xFF:
            raise RfidError(
                f"PN532响应命令错误: 0x{response[1]:02X},期望0x{(command + 1) & 0xFF:02X}"
            )
        return response[2:]

    def initialize(self) -> tuple[int, int, int, int]:
        self.wakeup()
        firmware = self.command(PN532_COMMAND_GET_FIRMWARE)
        if len(firmware) < 4:
            raise RfidError("PN532固件版本响应长度错误")
        self.command(PN532_COMMAND_SAM_CONFIGURATION, b"\x01\x14\x01")
        return tuple(firmware[:4])

    def detect_target(self, timeout: float = 1.2) -> Optional[PassiveTarget]:
        response = self.command(
            PN532_COMMAND_IN_LIST_PASSIVE_TARGET,
            b"\x01\x00",
            timeout=timeout,
        )
        if not response or response[0] == 0:
            return None
        if len(response) < 6:
            raise RfidError("寻卡响应长度错误")
        target_number = response[1]
        atqa = int.from_bytes(response[2:4], "big")
        sak = response[4]
        uid_length = response[5]
        if uid_length == 0 or len(response) < 6 + uid_length:
            raise RfidError("寻卡响应UID长度错误")
        return PassiveTarget(
            target_number,
            atqa,
            sak,
            bytes(response[6 : 6 + uid_length]),
        )

    def _exchange(self, target: PassiveTarget, payload: bytes) -> bytes:
        response = self.command(
            PN532_COMMAND_IN_DATA_EXCHANGE,
            bytes([target.target_number]) + payload,
            timeout=1.0,
        )
        if not response:
            raise RfidError("PN532数据交换响应为空")
        status = response[0]
        if status != 0:
            raise RfidError(f"MIFARE数据交换失败，状态码0x{status:02X}")
        return response[1:]

    def authenticate_block(
        self,
        target: PassiveTarget,
        block: int,
        key_a: bytes,
    ) -> None:
        if len(key_a) != 6:
            raise RfidError("MIFARE Key A必须是6字节")
        self._exchange(
            target,
            bytes([MIFARE_AUTHENTICATE_A, block]) + key_a + target.uid[-4:],
        )

    def read_block(self, target: PassiveTarget, block: int, key_a: bytes) -> bytes:
        self.authenticate_block(target, block, key_a)
        data = self._exchange(target, bytes([MIFARE_READ, block]))
        if len(data) != 16:
            raise RfidError(f"读取数据块长度错误: {len(data)}")
        return data

    def write_block(
        self,
        target: PassiveTarget,
        block: int,
        data: bytes,
        key_a: bytes,
    ) -> None:
        if len(data) != 16:
            raise RfidError("MIFARE数据块必须是16字节")
        self.authenticate_block(target, block, key_a)
        self._exchange(target, bytes([MIFARE_WRITE, block]) + data)


def parse_key(text: str) -> bytes:
    """解析12位十六进制Key A。"""

    compact = text.replace(" ", "").replace(":", "")
    if len(compact) != 12:
        raise RfidError("Key A必须是12个十六进制字符")
    try:
        return bytes.fromhex(compact)
    except ValueError as exc:
        raise RfidError("Key A不是有效的十六进制") from exc


def parse_number(text: str, name: str, maximum: int) -> int:
    try:
        value = int(text, 0)
    except ValueError as exc:
        raise RfidError(f"{name}不是有效数字") from exc
    if not 0 <= value <= maximum:
        raise RfidError(f"{name}必须在0到{maximum}之间")
    return value


def describe_target(target: PassiveTarget) -> str:
    kind = "MIFARE Classic" if target.is_mifare_classic else "非MIFARE Classic"
    return f"UID={target.uid_text}, ATQA=0x{target.atqa:04X}, SAK=0x{target.sak:02X}, {kind}"


class RfidWriterApp:
    """写卡工具图形界面。"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("AGV RFID 写卡工具")
        self.root.geometry("760x560")
        self.reader: Optional[Pn532Hsu] = None
        self.worker_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.port_var = tk.StringVar()
        self.tag_type_var = tk.StringVar(value="充电点")
        self.station_var = tk.StringVar(value="1")
        self.map_version_var = tk.StringVar(value="1")
        self.flags_var = tk.StringVar(value="0")
        self.sequence_var = tk.StringVar(value="1")
        self.key_var = tk.StringVar(value="FF FF FF FF FF FF")
        self.status_var = tk.StringVar(value="未连接")
        self._build_widgets()
        self.refresh_ports()
        self.root.after(100, self._poll_worker_queue)

    def _build_widgets(self) -> None:
        root = ttk.Frame(self.root, padding=12)
        root.pack(fill=tk.BOTH, expand=True)
        connection = ttk.LabelFrame(root, text="串口连接")
        connection.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(connection, text="端口").grid(row=0, column=0, padx=6, pady=6)
        self.port_box = ttk.Combobox(
            connection,
            textvariable=self.port_var,
            width=16,
            state="readonly",
        )
        self.port_box.grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=6)
        ttk.Button(connection, text="连接并初始化", command=self.connect).grid(row=0, column=3, padx=6)
        ttk.Button(connection, text="断开", command=self.disconnect).grid(row=0, column=4, padx=6)
        ttk.Label(connection, textvariable=self.status_var).grid(row=0, column=5, padx=8, sticky=tk.W)

        record = ttk.LabelFrame(root, text="AGV卡片记录")
        record.pack(fill=tk.X, pady=(0, 8))
        fields = [
            ("标签类型", self.tag_type_var, ["路径点", "普通站点", "充电点", "原点"]),
            ("位置ID", self.station_var, None),
            ("地图版本", self.map_version_var, None),
            ("标志位", self.flags_var, None),
            ("写入序号", self.sequence_var, None),
            ("Key A", self.key_var, None),
        ]
        for index, (label, variable, values) in enumerate(fields):
            row, column = divmod(index, 2)
            ttk.Label(record, text=label).grid(row=row, column=column * 2, padx=6, pady=5, sticky=tk.E)
            if values:
                widget = ttk.Combobox(record, textvariable=variable, values=values, state="readonly", width=18)
            else:
                widget = ttk.Entry(record, textvariable=variable, width=21)
            widget.grid(row=row, column=column * 2 + 1, padx=6, pady=5, sticky=tk.W)

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(actions, text="识别卡片", command=self.detect).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(actions, text="读取记录", command=self.read_record).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(actions, text="写入并校验", command=self.write_record).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(actions, text="清空日志", command=self.clear_log).pack(side=tk.RIGHT)

        self.log_text = tk.Text(root, height=20, width=96, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(self.log_text, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{timestamp} {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def clear_log(self) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_box.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _run_async(self, operation, success_message: str) -> None:
        if self.reader is None:
            messagebox.showwarning("未连接", "请先连接并初始化PN532")
            return

        def worker() -> None:
            try:
                result = operation()
                self.worker_queue.put(("ok", (success_message, result)))
            except Exception as exc:  # noqa: BLE001
                self.worker_queue.put(("error", str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _poll_worker_queue(self) -> None:
        try:
            while True:
                kind, value = self.worker_queue.get_nowait()
                if kind == "ok":
                    message, result = value
                    self.log(f"{message}: {result}")
                else:
                    self.log(f"错误: {value}")
                    messagebox.showerror("RFID操作失败", str(value))
        except queue.Empty:
            pass
        self.root.after(100, self._poll_worker_queue)

    def connect(self) -> None:
        self.disconnect()
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择串口", "请选择USB转3.3V串口的端口")
            return
        try:
            reader = Pn532Hsu(port)
            ic, version, revision, support = reader.initialize()
            self.reader = reader
            self.status_var.set("已连接")
            self.log(f"PN532初始化成功: IC=0x{ic:02X},版本={version}.{revision},支持=0x{support:02X}")
        except Exception as exc:  # noqa: BLE001
            self.status_var.set("连接失败")
            self.log(f"连接错误: {exc}")
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self) -> None:
        if self.reader is not None:
            self.reader.close()
            self.reader = None
        self.status_var.set("未连接")

    def _detect_target(self) -> PassiveTarget:
        if self.reader is None:
            raise RfidError("PN532未连接")
        target = self.reader.detect_target(timeout=2.0)
        if target is None:
            raise RfidError("没有检测到ISO14443-A标签")
        return target

    def detect(self) -> None:
        self._run_async(lambda: describe_target(self._detect_target()), "标签识别完成")

    def _make_record(self) -> AgvRecord:
        tag_types = {"路径点": 1, "普通站点": 2, "充电点": 3, "原点": 4}
        return AgvRecord(
            tag_types[self.tag_type_var.get()],
            parse_number(self.station_var.get(), "位置ID", 0xFFFF),
            parse_number(self.map_version_var.get(), "地图版本", 0xFFFF),
            parse_number(self.flags_var.get(), "标志位", 0xFFFF),
            parse_number(self.sequence_var.get(), "写入序号", 0xFFFF),
        )

    def _key(self) -> bytes:
        return parse_key(self.key_var.get())

    def read_record(self) -> None:
        try:
            key = self._key()
        except RfidError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def operation() -> str:
            target = self._detect_target()
            if not target.is_mifare_classic:
                raise RfidError(f"当前标签不是MIFARE Classic: {describe_target(target)}")
            records = []
            invalid_blocks = []
            for block in (DATA_BLOCK_PRIMARY, DATA_BLOCK_BACKUP):
                data = self.reader.read_block(target, block, key)  # type: ignore[union-attr]
                try:
                    records.append((block, AgvRecord.unpack(data)))
                except RfidError as exc:
                    invalid_blocks.append(f"块{block}:{exc}")
            if not records:
                raise RfidError("块4和块5都没有有效AGV记录")
            block, record = max(records, key=lambda item: item[1].sequence)
            suffix = f",其他={'；'.join(invalid_blocks)}" if invalid_blocks else ""
            return f"{describe_target(target)},块{block},类型={record.tag_type},位置ID={record.station_id},地图={record.map_version},序号={record.sequence}{suffix}"

        self._run_async(operation, "读取完成")

    def write_record(self) -> None:
        try:
            record = self._make_record()
            key = self._key()
        except (KeyError, RfidError) as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def operation() -> str:
            data = record.pack()
            target = self._detect_target()
            if not target.is_mifare_classic:
                raise RfidError(f"当前标签不是MIFARE Classic: {describe_target(target)}")
            for block in (DATA_BLOCK_PRIMARY, DATA_BLOCK_BACKUP):
                self.reader.write_block(target, block, data, key)  # type: ignore[union-attr]
            for block in (DATA_BLOCK_PRIMARY, DATA_BLOCK_BACKUP):
                if self.reader.read_block(target, block, key) != data:  # type: ignore[union-attr]
                    raise RfidError(f"块{block}读回内容不一致")
            return f"UID={target.uid_text},块4/5校验成功,位置ID={record.station_id},CRC=0x{crc16_ccitt(data[:14]):04X}"

        self._run_async(operation, "写卡完成")


def run_self_test() -> None:
    """运行不依赖硬件的记录格式自测。"""

    record = AgvRecord(3, 12, 2, 0x10, 7)
    packed = record.pack()
    assert len(packed) == RECORD_SIZE
    assert AgvRecord.unpack(packed) == record
    broken = bytearray(packed)
    broken[6] ^= 1
    try:
        AgvRecord.unpack(bytes(broken))
    except RfidError:
        pass
    else:
        raise AssertionError("损坏记录未被CRC拦截")
    print("RFID writer self-test passed")


def main() -> None:
    parser = argparse.ArgumentParser(description="PN532 HSU MIFARE Classic写卡工具")
    parser.add_argument("--self-test", action="store_true", help="运行记录格式自测")
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return
    root = tk.Tk()
    RfidWriterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
