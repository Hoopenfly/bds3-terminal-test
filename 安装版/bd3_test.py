#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""北斗三号终端测试工具.

功能:
  1. 选择串口与波特率并打开;
  2. 自动读取本机北斗地址(主动发 $CCICR 查询, 或从 IBD/ICP 输出中学习);
  3. 通信测试: 输入测试数据, 按设定周期(默认 65 秒)封装 $CCTCQ 帧发送;
  4. 依据 $BDFKI(发射反馈)与 $BDTCI(回环报文)判定成功/失败;
  5. 定位测试: 发 $CCRNS 只开 RMC、发 $CCRMO 设输出频度, 逐帧判定定位是否有效;
  6. 关闭/单次输出 $BDPWI 的设置指令;
  7. 实时统计两类测试的次数与成功率, 每次判定追加一行 CSV(失败另存一份).

协议依据: 《北斗三号民用终端通用数据接口要求》(2023-01-17 修正版)
  - 标准语句: $<类型标识>,<数据字段>...*<校验和><CR><LF>
  - 校验和: '$' 与 '*' 之间全部字符按字节异或, 两位十六进制大写
  - 串口: 8 数据位, 无校验, 1 停止位, 默认 115200 bps
  - CCTCQ: 收信方ID, 频点, 入站确认申请, 编码类别, 通信数据, 报文通信频度
  - BDFKI: 执行时间, 入站发射类型, 发射情况(Y/N), 失败原因, 剩余时间
  - BDTCI: 发信方ID, 收信方ID, 频点, 时间, 编码类别, 数据标识, 通信数据
  - IBD:   入站周计数, 周内秒, 入站频点, 用户地址, 入站序号, 收方地址, 入站类型

判定规则:
  - 一次发送为一个事务, 事务窗口内收到 FKI=Y 且收到地址、数据均匹配的 TCI, 判成功;
  - FKI=N 且失败原因不是"频率未到"(1)时, 立即判失败;
  - FKI=N 原因为 1 时设备通常会在剩余时间后自动重试, 继续等待;
  - 窗口结束仍未收到匹配 TCI, 判失败, 并给出具体原因.
"""

import argparse
import ctypes
import csv
import os
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, List, Optional, Tuple

#禁止写字节码缓存, 程序目录里不出现 __pycache__
sys.dont_write_bytecode = True

#程序所在目录: 打包成 exe 后取 exe 所在目录, 源码运行时取脚本所在目录
if getattr(sys, "frozen", False):
    _HERE = os.path.dirname(os.path.abspath(sys.executable))
else:
    _HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = os.path.join(_HERE, "lib")
if os.path.isdir(_LIB) and _LIB not in sys.path:
    sys.path.insert(0, _LIB)

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:  # 缺少 pyserial 时仍可启动界面, 但无法打开真实串口
    serial = None
    HAS_SERIAL = False

import tkinter as tk
from tkinter import ttk, messagebox


# ---------------------------------------------------------------------------
# 协议层
# ---------------------------------------------------------------------------

FAIL_REASONS = {
    "0": "未说明",
    "1": "频率未到(设备会按剩余时间自动重试)",
    "2": "发射抑制",
    "3": "发射静默",
    "4": "功率未锁定",
    "5": "未检测到 IC 模块信息",
    "6": "权限不足",
    "": "未说明",
}

#判定结果对应的中文说明, 写记录文件时使用
RESULT_TEXT = {"success": "成功", "fail": "失败", "interrupted": "中断"}

CSV_HEADER = ["序号", "类型", "时间", "本机地址", "测试数据", "间隔/频度(秒)",
              "判定", "失败标记", "失败原因", "详情", "用时(秒)"]

_SENTENCE_RE = re.compile(r"^\$(?P<body>[^*\r\n]*)\*(?P<chk>[0-9A-Fa-f]{2})$")


def xor_checksum(body: str) -> int:
    """计算北斗语句校验和: '$' 与 '*' 之间全部字符异或."""
    value = 0
    for ch in body:
        value ^= ord(ch)
    return value


def build_sentence(body: str) -> bytes:
    """由语句体(不含 '$'、'*hh')构造完整发送帧."""
    return ("$%s*%02X\r\n" % (body, xor_checksum(body))).encode("ascii")


def build_cctcq(dst_addr: str, data: str, freq: int = 2, ack: int = 1,
                coding: int = 2, interval: int = 0) -> bytes:
    """构造报文通信申请帧 $CCTCQ.

    默认参数与现场样例一致: 频点 2、不需确认、代码(ASCII)编码、单次发送.
    """
    body = "CCTCQ,%s,%d,%d,%d,%s,%d" % (dst_addr, freq, ack, coding, data, interval)
    return build_sentence(body)


def build_icr_query() -> bytes:
    """构造本机 IC 模块信息查询帧 $CCICR,0,00, 用于读取本机地址."""
    return build_sentence("CCICR,0,00")


def build_ccrns_rmc() -> bytes:
    """构造"仅开启 RMC 输出"的设置帧.

    现场实测指令, 文档未收录: $CCRNS,0,0,0,0,1,0,8.
    7 个字段依次是 GGA/GSV/GLL/GSA/RMC/ZDA/配置位, 第 5 位为 RMC.
    末位只接受 0 或 8 且两者都输出 1 帧/秒, 不是频度; 频度用 build_rmo 设置.
    """
    return build_sentence("CCRNS,0,0,0,0,1,0,8")


def build_rmo(target: str, mode: int, freq: Optional[int] = None) -> bytes:
    """构造输出语句开关帧 $CCRMO.

    文档 4.1.21 RMO: 模式 1-关闭指定语句, 2-打开指定语句,
    3-关闭全部, 4-打开全部; 频度 1 表示 1 秒一帧, 0 按文档是单次输出
    (RMC 实测传 0 会停止输出, 见 build_rmc_freq).
    """
    body = "CCRMO,%s,%d" % (target, mode)
    if freq is not None:
        body += ",%d" % freq
    return build_sentence(body)


def build_rmc_freq(freq: int) -> bytes:
    """设置 RMC 输出频度: $CCRMO,RMC,2,<秒>.

    文档说频度 0 是"单次输出", 但本机实测传 0 后一帧都不再输出(等于停止),
    所以界面只接受 1~3600, 不接受 0.
    """
    return build_rmo("RMC", 2, freq)


def build_pwi_off() -> bytes:
    """关闭 PWI 波束状态输出: $CCRMO,PWI,1,0."""
    return build_rmo("PWI", 1, 0)


def build_pwi_once() -> bytes:
    """让设备单次输出一帧 PWI: $CCRMO,PWI,2,0 (打开+频度 0)."""
    return build_rmo("PWI", 2, 0)



@dataclass
class Sentence:
    """解析后的北斗语句."""

    raw: str
    type_id: str
    fields: List[str]
    checksum_ok: bool

    def field(self, index: int) -> str:
        """按序号取字段, 越界返回空串."""
        if 0 <= index < len(self.fields):
            return self.fields[index]
        return ""


def parse_sentence(raw: bytes) -> Optional[Sentence]:
    """解析一行原始字节, 无法识别时返回 None."""
    text = raw.decode("ascii", "replace").strip("\r\n")
    match = _SENTENCE_RE.match(text)
    if not match:
        return None
    body = match.group("body")
    try:
        expect = int(match.group("chk"), 16)
    except ValueError:
        return None
    return Sentence(
        raw=text,
        type_id=body.split(",", 1)[0],
        fields=body.split(","),
        checksum_ok=(xor_checksum(body) == expect),
    )


# ---------------------------------------------------------------------------
# 测试事务与判定
# ---------------------------------------------------------------------------

@dataclass
class TxRecord:
    """一次发送对应的判定事务."""

    seq: int
    addr: str
    data: str
    sent_at: float
    wall_time: str = ""  #发送时刻的本地时间, 写记录文件用
    fki: List[Tuple[str, str, str, str]] = field(default_factory=list)
    tci_seen: bool = False
    tci_mismatch: str = ""
    result: str = ""  # success / fail / interrupted
    fail_reason: str = ""
    finalized: bool = False


class TestSession:
    """通信自发自收判定状态机, 与界面、串口解耦, 便于单元测试."""

    def __init__(self, addr: str, data: str, wait_s: float = 60.0,
                 on_finish: Optional[Callable[[TxRecord, str, str, float], None]] = None) -> None:
        self.addr = addr
        self.data = data
        self.wait_s = wait_s
        self.on_finish = on_finish
        self.seq = 0
        self.sent_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.current: Optional[TxRecord] = None

    # -- 发送 ---------------------------------------------------------------
    def start_tx(self, now: float) -> Tuple[TxRecord, bytes, List[Tuple[str, str]]]:
        """开始一次发送; 若上一事务未结束则先按超时收尾."""
        events: List[Tuple[str, str]] = []
        if self.current is not None and not self.current.finalized:
            self._finish(self.current, "fail", "上一次发送的等待窗口已结束", now)
            events.append(("err", "第 %d 次: 上一次发送等待超时, 判失败" % self.current.seq))

        self.seq += 1
        self.sent_count += 1
        rec = TxRecord(seq=self.seq, addr=self.addr, data=self.data, sent_at=now)
        self.current = rec
        frame = build_cctcq(self.addr, self.data)
        return rec, frame, events

    # -- 接收 ---------------------------------------------------------------
    def on_sentence(self, sent: Sentence, now: float) -> List[Tuple[str, str]]:
        """处理一条已通过校验的语句, 返回需要记录的事件."""
        events: List[Tuple[str, str]] = []
        rec = self.current
        if rec is None or rec.finalized:
            return events

        if sent.type_id == "BDFKI":
            events.extend(self._on_fki(sent, rec, now))
        elif sent.type_id == "BDTCI":
            events.extend(self._on_tci(sent, rec, now))
        return events

    def _on_fki(self, sent: Sentence, rec: TxRecord,
                now: float) -> List[Tuple[str, str]]:
        events: List[Tuple[str, str]] = []
        if sent.field(2) != "TCQ":
            return events  # 只关心报文通信申请的反馈
        status = sent.field(3).upper()
        reason = sent.field(4)
        remain = sent.field(5)
        rec.fki.append((sent.field(1), status, reason, remain))

        if status == "Y":
            events.append(("info", "FKI 发射成功, 等待回环报文(第 %d 次)" % rec.seq))
            return events

        reason_text = FAIL_REASONS.get(reason, "未知原因 %s" % reason)
        events.append(("warn", "FKI 发射失败: 原因 %s(%s), 剩余 %s 秒"
                       % (reason or "?", reason_text, remain or "?")))
        if reason != "1":
            # 频率未到之外的原因不会自动重试, 直接判失败
            self._finish(rec, "fail", "发射失败: %s" % reason_text, now)
            events.append(("err", "第 %d 次: 判失败(%s)" % (rec.seq, reason_text)))
        return events

    def _on_tci(self, sent: Sentence, rec: TxRecord,
                now: float) -> List[Tuple[str, str]]:
        events: List[Tuple[str, str]] = []
        tx_addr = sent.field(1)
        rx_addr = sent.field(2)
        coding = sent.field(5)
        data = sent.field(7)

        addr_ok = (tx_addr == rec.addr and rx_addr == rec.addr)
        data_ok = (data == rec.data)
        if addr_ok and data_ok:
            rec.tci_seen = True
            self._finish(rec, "success", "", now)
            events.append(("ok", "第 %d 次: 回环成功, 收发地址均为 %s, 数据一致" %
                           (rec.seq, rec.addr)))
        else:
            rec.tci_mismatch = "发信=%s 收信=%s 数据=%r" % (tx_addr, rx_addr, data)
            events.append(("warn", "收到 TCI 但与本次发送不匹配(%s), 继续等待" %
                           rec.tci_mismatch))
        return events

    # -- 超时 ---------------------------------------------------------------
    def tick(self, now: float) -> List[Tuple[str, str]]:
        """超时检查, 窗口结束仍未成功则判失败."""
        events: List[Tuple[str, str]] = []
        rec = self.current
        if rec is None or rec.finalized:
            return events
        if now - rec.sent_at < self.wait_s:
            return events

        if rec.tci_seen:
            return events  # 理论不会走到这里(success 已收尾)
        if rec.fki:
            last_status, last_reason = rec.fki[-1][1], rec.fki[-1][2]
            if last_status == "Y":
                reason = "入站成功但未收到匹配的回环报文"
            elif last_reason == "1":
                reason = "发射失败: 频率未到(等待自动重试超时)"
            else:
                reason = "发射失败: %s" % FAIL_REASONS.get(last_reason, last_reason)
        else:
            reason = "未收到 FKI 发射反馈"
        if rec.tci_mismatch:
            reason += "; 收到的 TCI 不匹配(%s)" % rec.tci_mismatch

        self._finish(rec, "fail", reason, now)
        events.append(("err", "第 %d 次: 判失败(%s)" % (rec.seq, reason)))
        return events

    def stop(self) -> List[Tuple[str, str]]:
        """停止测试: 未判定事务标记为中断, 不计入成功/失败."""
        events: List[Tuple[str, str]] = []
        rec = self.current
        if rec is not None and not rec.finalized:
            self._finish(rec, "interrupted", "", time.monotonic())
            events.append(("warn", "第 %d 次发送尚未判定, 已按中断处理(不计失败)"
                           % rec.seq))
        return events

    def _finish(self, rec: TxRecord, result: str, reason: str, now: float) -> None:
        rec.result = result
        rec.fail_reason = reason
        rec.finalized = True
        if result == "success":
            self.success_count += 1
        elif result == "fail":
            self.fail_count += 1
        if self.on_finish is not None:
            self.on_finish(rec, result, reason, now)

    # -- 统计 ---------------------------------------------------------------
    @property
    def pending_count(self) -> int:
        rec = self.current
        return 1 if (rec is not None and not rec.finalized) else 0

    @property
    def success_rate(self) -> Optional[float]:
        if self.sent_count <= 0:
            return None
        return self.success_count * 100.0 / self.sent_count


# ---------------------------------------------------------------------------
# 定位测试
# ---------------------------------------------------------------------------

@dataclass
class PosRecord:
    """一帧 RMC 对应的定位判定记录."""

    seq: int
    wall_time: str
    status: str = ""  #RMC 定位状态 A/V
    lat: str = ""
    lon: str = ""
    mode: str = ""  #RMC 模式指示
    valid: bool = False
    fail_reason: str = ""
    raw: str = ""


def parse_rmc(sent: Sentence) -> Optional[PosRecord]:
    """从 RMC 帧提取定位状态与经纬度; 不是 RMC 返回 None.

    RMC 字段(文档表 5-19): 1 时间, 2 定位状态, 3 纬度, 4 纬度方向,
    5 经度, 6 经度方向, 7 速度, 8 航向, 9 日期, 10 磁偏角,
    11 磁偏方向, 12 模式指示.
    """
    if not sent.type_id.upper().endswith("RMC"):
        return None
    return PosRecord(seq=0, wall_time="", status=sent.field(2).upper(),
                     lat=sent.field(3).strip(), lon=sent.field(5).strip(),
                     mode=sent.field(12).upper(), raw=sent.raw)


class PositionSession:
    """定位测试判定: 每收到一帧 RMC 判一次, 与界面解耦, 便于单元测试."""

    def __init__(self,
                 on_finish: Optional[Callable[[PosRecord, bool, str, float], None]] = None) -> None:
        self.on_finish = on_finish
        self.seq = 0
        self.total = 0
        self.valid_count = 0
        self.invalid_count = 0
        self.last_seen: float = 0.0  #最近一帧 RMC 的时刻, 用于无数据告警
        self.last: Optional[PosRecord] = None

    def on_rmc(self, sent: Sentence, now: float) -> List[Tuple[str, str]]:
        """处理一帧 RMC, 返回需要记录的事件."""
        events: List[Tuple[str, str]] = []
        rec = parse_rmc(sent)
        if rec is None:
            return events

        self.seq += 1
        self.total += 1
        rec.seq = self.seq
        rec.wall_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.last_seen = now
        self.last = rec

        coord_text = "%s%s/%s" % (rec.lat, rec.lon, rec.status)
        if rec.status != "A":
            reason = "定位状态为 %s(无效定位)" % (rec.status or "空")
        elif not rec.lat or not rec.lon:
            reason = "RMC 中经纬度为空"
        elif rec.mode == "N":
            reason = "模式指示 N(无效)"
        else:
            reason = ""

        rec.valid = not reason
        rec.fail_reason = reason
        if rec.valid:
            self.valid_count += 1
            events.append(("ok", "第 %d 帧定位有效: 纬度 %s, 经度 %s, 状态 A"
                           % (rec.seq, rec.lat, rec.lon)))
        else:
            self.invalid_count += 1
            events.append(("err", "第 %d 帧定位无效: %s" % (rec.seq, reason)))
        if self.on_finish is not None:
            self.on_finish(rec, rec.valid, reason, now)
        return events

    # -- 统计 ---------------------------------------------------------------
    @property
    def success_rate(self) -> Optional[float]:
        if self.total <= 0:
            return None
        return self.valid_count * 100.0 / self.total


# ---------------------------------------------------------------------------
# 串口读写
# ---------------------------------------------------------------------------

class ReaderThread(threading.Thread):
    """串口读线程: 按行切分后放入队列, 由界面线程消费."""

    def __init__(self, ser, out_queue: "queue.Queue") -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.out = out_queue
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        buf = bytearray()
        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting if waiting > 0 else 1)
            except Exception as exc:  # 串口被拔出或关闭
                self.out.put(("error", str(exc)))
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                line = bytes(buf[:idx]).strip(b"\r")
                del buf[:idx + 1]
                if line:
                    self.out.put(("line", line))
            if len(buf) > 8192:  # 防御: 无换行的异常数据不能无限增长
                buf.clear()
                self.out.put(("error", "接收缓冲区溢出, 已丢弃未成帧数据"))


def list_ports() -> List[Tuple[str, str]]:
    """返回 [(设备名, 描述)]."""
    if not HAS_SERIAL:
        return []
    return [(p.device, p.description or "") for p in serial.tools.list_ports.comports()]


# ---------------------------------------------------------------------------
# 模拟终端(无硬件时验证界面与判定链路)
# ---------------------------------------------------------------------------

class SimulatedPort:
    """模拟北斗终端: 收到 CCTCQ 后按压缩后的时序回 FKI/IBD/TCI.

    第三次发送模拟现场日志中的"N(频率未到) -> 自动重试 -> Y"过程.
    """

    def __init__(self, addr: str = "1881210", scaling: float = 0.1) -> None:
        self.addr = addr
        self.scaling = scaling
        self.rx: "queue.Queue[bytes]" = queue.Queue()
        self.is_open = True
        self._lock = threading.Lock()
        self._timers: List[threading.Timer] = []
        self._send_count = 0
        self._ibd_seq = 28
        self._rmc_freq = 8
        self._rmc_seq = 0
        self._rmc_gen = 0

    # -- pyserial 兼容接口 --------------------------------------------------
    @property
    def in_waiting(self) -> int:
        return self.rx.qsize()

    def read(self, size: int = 1, timeout: Optional[float] = None) -> bytes:
        try:
            item = self.rx.get(timeout=0.1 if timeout is None else timeout)
            return item
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> int:
        text = data.decode("ascii", "ignore").strip()
        match = re.match(r"^\$CCTCQ,([^,]+),(\d+),(\d+),(\d+),([^,]+),(\d+)\*", text)
        if match:
            self._send_count += 1
            dst = match.group(1)
            payload = match.group(5)
            seq = self._send_count
            self._later(0.05, self._push_ibd)
            if seq % 3 == 0:
                self._later(0.1, lambda: self._push_fki("N", "1", "10"))
                self._later(11.0, lambda: self._push_fki("Y", "0", "0"))
                self._later(11.6, lambda: self._push_tci(dst, payload))
            else:
                self._later(0.1, lambda: self._push_fki("Y", "0", "0"))
                self._later(1.2, lambda: self._push_tci(dst, payload))
        elif text.startswith("$CCICR"):
            self._later(0.2, self._push_icp)
        elif text.startswith("$CCRNS,"):
            #仅开启 RMC 输出, 频度由后续 $CCRMO,RMC 设置
            self._start_rmc()
        elif text.startswith("$CCRMO,"):
            if "PWI" in text:
                self._later(0.2, self._push_pwi)
            elif "RMC" in text:
                parts = text.split("*")[0].split(",")
                try:
                    freq = int(parts[-1])
                except (ValueError, IndexError):
                    freq = 1
                self._set_rmc_freq(freq)
        return len(data)

    def close(self) -> None:
        self.is_open = False
        with self._lock:
            for timer in self._timers:
                timer.cancel()
            self._timers.clear()

    # -- 内部 ---------------------------------------------------------------
    def _later(self, delay: float, func) -> None:
        timer = threading.Timer(delay * self.scaling, func)
        timer.daemon = True
        with self._lock:
            self._timers.append(timer)
        timer.start()

    def _push(self, frame: bytes) -> None:
        if self.is_open:
            self.rx.put(frame)

    def _push_fki(self, status: str, reason: str, remain: str) -> None:
        stamp = datetime.now(timezone.utc).strftime("%H%M%S")
        self._push(build_sentence("BDFKI,%s,TCQ,%s,%s,%s" % (stamp, status, reason, remain)))

    def _push_ibd(self) -> None:
        self._ibd_seq += 1
        self._push(build_sentence("BDIBD,1090,291051.594,1,%s,%d,%s,0"
                                  % (self.addr, self._ibd_seq, self.addr)))

    def _push_tci(self, dst: str, payload: str) -> None:
        stamp = datetime.now().strftime("%H%M%S")
        self._push(build_sentence("BDTCI,%s,%s,2,%s,2,0,%s"
                                  % (self.addr, dst, stamp, payload)))

    def _push_icp(self) -> None:
        self._push(build_sentence(
            "BDICP,%s,0,1,3,0,N,3,N,0,0,0,3,1,2,0,0,0,0,0,0,0,0" % self.addr))

    # -- 定位模拟 -----------------------------------------------------------
    def _start_rmc(self) -> None:
        """模拟"仅开 RMC": 立即输出一帧, 之后按 1 秒周期输出."""
        self._rmc_freq = 1
        self._rmc_seq = 0
        self._rmc_gen += 1
        self._push_rmc()
        self._schedule_next_rmc(self._rmc_gen)

    def _set_rmc_freq(self, freq: int) -> None:
        """模拟 $CCRMO,RMC,2,<秒>: 0 只输出一帧, 其它值按秒周期输出.

        真机传 0 是停止输出(见 build_rmc_freq), 界面不接受 0, 这条分支只供构造测试.
        """
        self._rmc_gen += 1
        gen = self._rmc_gen
        if freq <= 0:
            self._push_rmc()
            return
        self._rmc_freq = freq
        self._push_rmc()
        self._schedule_next_rmc(gen)

    def _schedule_next_rmc(self, gen: int) -> None:
        if not self.is_open:
            return
        timer = threading.Timer(self._rmc_freq * self.scaling, self._rmc_tick, (gen,))
        timer.daemon = True
        with self._lock:
            self._timers.append(timer)
        timer.start()

    def _rmc_tick(self, gen: int) -> None:
        if not self.is_open or gen != self._rmc_gen:
            return
        self._push_rmc()
        self._schedule_next_rmc(gen)

    def _push_rmc(self) -> None:
        """前两帧状态 V(未定位)且经纬度为空, 之后状态 A 并带经纬度."""
        self._rmc_seq += 1
        stamp = datetime.now().strftime("%H%M%S") + ".00"
        date = datetime.now().strftime("%d%m%y")
        if self._rmc_seq <= 2:
            body = "GNRMC,%s,V,,,,,,,%s,,,N" % (stamp, date)
        else:
            body = ("GNRMC,%s,A,3140.12345,N,11848.56789,E,0.0,0.0,%s,,,A"
                    % (stamp, date))
        self._push(build_sentence(body))

    def _push_pwi(self) -> None:
        stamp = datetime.now().strftime("%H%M%S") + ".00"
        self._push(build_sentence("BDPWI,%s,0,0,0,0,0" % stamp))


# ---------------------------------------------------------------------------
# 界面
# ---------------------------------------------------------------------------

FONT_UI = ("Microsoft YaHei UI", 10)
FONT_LOG = ("Consolas", 10)


class App:
    """主窗口与交互逻辑."""

    def __init__(self, root: tk.Tk, simulate: bool = False) -> None:
        self.root = root
        self.simulate = simulate
        self.ser = None
        self.reader: Optional[ReaderThread] = None
        self.session: Optional[TestSession] = None
        self.running = False
        self.next_send_at = 0.0
        self.rx_queue: "queue.Queue" = queue.Queue()
        self.write_lock = threading.Lock()
        self.icr_deadline = 0.0
        self.csv_fp = None
        self.csv_writer = None
        self.csv_path = ""
        self.csv_fail = False
        self.pos_session: Optional[PositionSession] = None
        self.fail_fp = None
        self.fail_writer = None
        self.pos_last_seen = 0.0
        self.pos_warned_at = 0.0
        self.ack_shown = False

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._tick)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        if simulate:
            self.sim_var.set(True)
            self._log("模拟模式: 不打开真实串口, 使用内置终端模拟器", "info")

    # -- 界面搭建 -----------------------------------------------------------
    def _build_ui(self) -> None:
        self.root.title("北斗三号终端测试工具")
        self.root.geometry("1200x820")
        self.root.minsize(1100, 720)

        style = ttk.Style()
        style.configure("TLabel", font=FONT_UI)
        style.configure("TButton", font=FONT_UI)
        style.configure("TCombobox", font=FONT_UI)
        style.configure("TEntry", font=FONT_UI)
        style.configure("TCheckbutton", font=FONT_UI)

        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        # 串口行
        row1 = ttk.Frame(main)
        row1.pack(fill="x", pady=(0, 6))
        ttk.Label(row1, text="串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(row1, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.pack(side="left", padx=(4, 2))
        self.btn_refresh = ttk.Button(row1, text="刷新", width=6, command=self.refresh_ports)
        self.btn_refresh.pack(side="left", padx=(0, 14))
        ttk.Label(row1, text="波特率:").pack(side="left")
        self.baud_var = tk.StringVar(value="115200")
        self.baud_box = ttk.Combobox(row1, textvariable=self.baud_var, width=10,
                                     values=["4800", "9600", "19200", "38400",
                                             "57600", "115200"], state="readonly")
        self.baud_box.pack(side="left", padx=(4, 14))
        self.btn_open = ttk.Button(row1, text="打开串口", width=10, command=self.toggle_port)
        self.btn_open.pack(side="left")

        # 地址与数据行
        row2 = ttk.Frame(main)
        row2.pack(fill="x", pady=(0, 6))
        ttk.Label(row2, text="本机地址:").pack(side="left")
        self.addr_var = tk.StringVar(value="")
        self.addr_entry = ttk.Entry(row2, textvariable=self.addr_var, width=12)
        self.addr_entry.pack(side="left", padx=(4, 4))
        self.btn_read_addr = ttk.Button(row2, text="读取地址", width=9,
                                        command=self.read_address)
        self.btn_read_addr.pack(side="left", padx=(0, 4))
        self.addr_state = ttk.Label(row2, text="", foreground="#888888")
        self.addr_state.pack(side="left", padx=(0, 18))
        self.addr_entry.bind("<KeyRelease>", lambda _e: self.addr_state.configure(text="手动输入"))
        ttk.Label(row2, text="测试数据:").pack(side="left")
        self.data_var = tk.StringVar(value="0123456789ABCDEF")
        self.data_entry = ttk.Entry(row2, textvariable=self.data_var, width=24)
        self.data_entry.pack(side="left", padx=(4, 0))

        # 测试模式与参数行
        row3 = ttk.Frame(main)
        row3.pack(fill="x", pady=(0, 6))
        ttk.Label(row3, text="测试模式:").pack(side="left")
        self.mode_var = tk.StringVar(value="comm")
        for text, value in [("仅通信", "comm"), ("仅定位", "pos"), ("两者都", "both")]:
            ttk.Radiobutton(row3, text=text, value=value, variable=self.mode_var).pack(
                side="left", padx=(6, 0))
        ttk.Label(row3, text="通信间隔(秒):").pack(side="left", padx=(18, 0))
        self.interval_var = tk.StringVar(value="65")
        self.interval_entry = ttk.Entry(row3, textvariable=self.interval_var, width=6)
        self.interval_entry.pack(side="left", padx=(4, 14))
        ttk.Label(row3, text="RMC频度(秒):").pack(side="left")
        self.pos_freq_var = tk.StringVar(value="1")
        self.pos_freq_entry = ttk.Entry(row3, textvariable=self.pos_freq_var, width=6)
        self.pos_freq_entry.pack(side="left", padx=(4, 0))

        # 开关与按钮行
        row4 = ttk.Frame(main)
        row4.pack(fill="x", pady=(0, 8))
        self.sim_var = tk.BooleanVar(value=False)
        self.chk_sim = ttk.Checkbutton(row4, text="模拟模式", variable=self.sim_var,
                                       command=self.on_sim_toggle)
        self.chk_sim.pack(side="left", padx=(0, 12))
        self.btn_start = ttk.Button(row4, text="开始测试", width=11, command=self.start_test)
        self.btn_start.pack(side="left")
        self.btn_stop = ttk.Button(row4, text="停止测试", width=11, command=self.stop_test,
                                   state="disabled")
        self.btn_stop.pack(side="left", padx=(8, 0))
        self.btn_pwi_off = ttk.Button(row4, text="关闭PWI", width=9, command=self.send_pwi_off)
        self.btn_pwi_off.pack(side="left", padx=(16, 0))
        self.btn_pwi_once = ttk.Button(row4, text="单次PWI", width=9, command=self.send_pwi_once)
        self.btn_pwi_once.pack(side="left", padx=(6, 0))

        # 统计行
        stat = ttk.LabelFrame(main, text="统计", padding=8)
        stat.pack(fill="x", pady=(0, 8))
        self.var_sent = tk.StringVar(value="0")
        self.var_ok = tk.StringVar(value="0")
        self.var_fail = tk.StringVar(value="0")
        self.var_rate = tk.StringVar(value="--")
        self.var_pending = tk.StringVar(value="0")
        self.var_pos_total = tk.StringVar(value="0")
        self.var_pos_ok = tk.StringVar(value="0")
        self.var_pos_bad = tk.StringVar(value="0")
        self.var_pos_rate = tk.StringVar(value="--")

        def stat_cell(row: int, col: int, title: str, var: tk.StringVar, color: str) -> None:
            pad = (10, 2) if col else (0, 2)
            ttk.Label(stat, text=title + ":", font=FONT_UI).grid(
                row=row, column=col, sticky="e", padx=pad, pady=(6 if row else 0, 0))
            ttk.Label(stat, textvariable=var, font=("Microsoft YaHei UI", 13, "bold"),
                      foreground=color).grid(
                row=row, column=col + 1, sticky="w", pady=(6 if row else 0, 0))

        ttk.Label(stat, text="通信:", font=("Microsoft YaHei UI", 10, "bold")).grid(
            row=0, column=0, sticky="w")
        for i, (title, var, color) in enumerate([
                ("发送次数", self.var_sent, "#1a56db"),
                ("成功次数", self.var_ok, "#0e9f6e"),
                ("失败次数", self.var_fail, "#e02424"),
                ("成功率", self.var_rate, "#1a56db"),
                ("待判定", self.var_pending, "#888888")]):
            stat_cell(0, 1 + i * 2, title, var, color)

        ttk.Label(stat, text="定位:", font=("Microsoft YaHei UI", 10, "bold")).grid(
            row=1, column=0, sticky="w", pady=(6, 0))
        for i, (title, var, color) in enumerate([
                ("判定帧数", self.var_pos_total, "#1a56db"),
                ("有效帧数", self.var_pos_ok, "#0e9f6e"),
                ("无效帧数", self.var_pos_bad, "#e02424"),
                ("成功率", self.var_pos_rate, "#1a56db")]):
            stat_cell(1, 1 + i * 2, title, var, color)

        # 日志区
        log_frame = ttk.LabelFrame(main, text="收发日志", padding=6)
        log_frame.pack(fill="both", expand=True)
        self.log_text = tk.Text(log_frame, font=FONT_LOG, wrap="none", state="disabled",
                                bg="#0f172a", fg="#cbd5e1", insertbackground="#cbd5e1")
        scroll_y = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scroll_x = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log_text.xview)
        self.log_text.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        for tag, color in [("tx", "#60a5fa"), ("rx", "#cbd5e1"), ("ok", "#34d399"),
                           ("warn", "#fbbf24"), ("err", "#f87171"),
                           ("info", "#94a3b8")]:
            self.log_text.tag_configure(tag, foreground=color)

        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(main, textvariable=self.status_var, foreground="#666666").pack(
            fill="x", pady=(6, 0))

    # -- 串口 ---------------------------------------------------------------
    def refresh_ports(self) -> None:
        ports = list_ports()
        values = ["%s  %s" % (dev, desc) for dev, desc in ports]
        self.port_box.configure(values=values)
        if values and not self.port_var.get():
            self.port_var.set(values[0])
        if not values:
            self.port_var.set("")
        self.status_var.set("找到 %d 个串口" % len(ports) if ports else "未找到串口")

    def selected_port(self) -> str:
        text = self.port_var.get().strip()
        return text.split()[0] if text else ""

    def toggle_port(self) -> None:
        if self.ser is not None:
            self.close_port()
        else:
            self.open_port()

    def open_port(self) -> bool:
        if self.sim_var.get():
            self.ser = SimulatedPort(self.addr_var.get().strip() or "1881210")
            self._start_reader()
            self.btn_open.configure(text="关闭串口")
            self.status_var.set("模拟终端已启动")
            self._set_port_controls(False)
            self._log("模拟终端已启动", "info")
            return True

        if not HAS_SERIAL:
            messagebox.showerror("缺少依赖", "未找到 pyserial, 请确认 lib 目录存在。")
            return False
        port = self.selected_port()
        if not port:
            messagebox.showwarning("提示", "请先选择串口。")
            return False
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     timeout=0.1)
        except Exception as exc:
            self.ser = None
            messagebox.showerror("打开串口失败", "%s\n\n%s" % (port, exc))
            return False
        self._start_reader()
        self.btn_open.configure(text="关闭串口")
        self.status_var.set("%s 已打开" % port)
        self._set_port_controls(False)
        self._log("串口 %s 已打开, 波特率 %d, 8N1" % (port, baud), "info")
        return True

    def _start_reader(self) -> None:
        self.reader = ReaderThread(self.ser, self.rx_queue)
        self.reader.start()

    def close_port(self) -> None:
        if self.running:
            self.stop_test()  #关闭串口时自动停止测试
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.btn_open.configure(text="打开串口")
        self.status_var.set("串口已关闭")
        self._set_port_controls(True)
        self._log("串口已关闭", "info")

    def _set_port_controls(self, enabled: bool) -> None:
        state = "readonly" if enabled else "disabled"
        self.port_box.configure(state=state)
        self.baud_box.configure(state=state)
        self.btn_refresh.configure(state="normal" if enabled else "disabled")
        self.chk_sim.configure(state="normal" if enabled else "disabled")

    def on_sim_toggle(self) -> None:
        if self.ser is not None:
            self.sim_var.set(not self.sim_var.get())
            messagebox.showinfo("提示", "请先关闭当前串口再切换模式。")
            return
        if self.sim_var.get():
            self.addr_state.configure(text="模拟地址 1881210", foreground="#888888")
        else:
            self.addr_state.configure(text="")

    # -- PWI 波束状态 -------------------------------------------------------
    def send_pwi_off(self) -> None:
        """关闭 PWI 输出."""
        self._send_setting(build_pwi_off(), "关闭 PWI 输出")

    def send_pwi_once(self) -> None:
        """让设备单次输出一帧 PWI."""
        self._send_setting(build_pwi_once(), "单次输出 PWI")

    def _send_setting(self, frame: bytes, desc: str) -> None:
        if self.ser is None and not self.open_port():
            return
        try:
            self._write(frame)
        except Exception as exc:
            self._log("%s 发送失败: %s" % (desc, exc), "err")
            return
        self._log("%s: %s" % (desc, frame.decode("ascii").strip()), "tx")

    # -- 地址读取 -----------------------------------------------------------
    def read_address(self) -> None:
        if self.ser is None:
            messagebox.showwarning("提示", "请先打开串口。")
            return
        frame = build_icr_query()
        try:
            self._write(frame)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))
            return
        self.icr_deadline = time.monotonic() + 5.0
        self.addr_state.configure(text="查询中...", foreground="#888888")
        self._log("发送地址查询 %s" % frame.decode().strip(), "tx")

    def _learn_address(self, sent: Sentence) -> None:
        addr = ""
        if sent.type_id == "BDIBD" and len(sent.fields) >= 5:
            addr = sent.field(4)  # IBD: 用户地址
        elif sent.type_id in ("BDIC", "BDICP") and len(sent.fields) >= 2:
            addr = sent.field(1)  # ICP: 用户地址(ID 号)
        if not addr or not addr.isdigit():
            return
        if self.icr_deadline > 0:
            self.icr_deadline = 0.0
            self.addr_var.set(addr)
            self.addr_state.configure(text="已读取", foreground="#0e9f6e")
            self._log("读取到本机地址 %s" % addr, "ok")
        elif not self.addr_var.get().strip():
            self.addr_var.set(addr)
            self.addr_state.configure(text="自动获取", foreground="#0e9f6e")
            self._log("从 %s 帧自动获取本机地址 %s" % (sent.type_id, addr), "info")

    # -- 测试控制 -----------------------------------------------------------
    def _validate(self) -> Optional[Tuple[str, str, int, str, int]]:
        mode = self.mode_var.get()
        addr = self.addr_var.get().strip()
        data = self.data_var.get().strip()
        if mode in ("comm", "both"):
            if not re.fullmatch(r"\d{5,12}", addr):
                messagebox.showwarning("参数错误", "本机地址应为 5~12 位数字, 例如 1881210。")
                return None
            if not (1 <= len(data) <= 200):
                messagebox.showwarning("参数错误", "测试数据长度应为 1~200 个字符。")
                return None
            bad = [ch for ch in data if not (0x20 <= ord(ch) <= 0x7E) or ch in ",*$"]
            if bad:
                messagebox.showwarning("参数错误",
                                       "测试数据仅支持 ASCII 可打印字符, 且不能包含 , * $。")
                return None
        try:
            interval = int(self.interval_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "通信间隔应为整数秒。")
            return None
        if not 5 <= interval <= 3600:
            messagebox.showwarning("参数错误", "通信间隔应在 5~3600 秒之间。")
            return None
        try:
            pos_freq = int(self.pos_freq_var.get())
        except ValueError:
            messagebox.showwarning("参数错误", "RMC 频度应为整数。")
            return None
        if not 1 <= pos_freq <= 3600:
            messagebox.showwarning(
                "参数错误",
                "RMC 频度应为 1~3600 秒。0 按文档是单次输出，但实测本机会停掉 RMC 输出，"
                "定位测试会一帧都收不到，所以不接受 0。")
            return None
        return addr, data, interval, mode, pos_freq

    def start_test(self) -> None:
        if self.running:
            return
        if self.ser is None and not self.open_port():
            return
        params = self._validate()
        if params is None:
            return
        addr, data, interval, mode, pos_freq = params
        wait_s = min(60.0, max(5.0, interval - 5.0))
        self._reset_stats()  #重新开始时清零两组统计, 不残留上一次的数字
        self.csv_fail = False
        self.ack_shown = False
        self._open_csv()
        self.session = None
        self.pos_session = None
        if mode in ("comm", "both"):
            self.session = TestSession(addr, data, wait_s, on_finish=self._write_csv)
            self.next_send_at = time.monotonic()
        if mode in ("pos", "both"):
            self.pos_session = PositionSession(on_finish=self._write_pos_csv)
            self.pos_last_seen = time.monotonic()
            self.pos_warned_at = 0.0
        self.running = True
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.addr_entry.configure(state="disabled")
        self.data_entry.configure(state="disabled")
        self.interval_entry.configure(state="disabled")
        self.pos_freq_entry.configure(state="disabled")
        if mode in ("comm", "both"):
            self._log("开始通信测试: 地址 %s, 数据 %s, 间隔 %d 秒, 判定窗口 %.0f 秒"
                      % (addr, data, interval, wait_s), "info")
        if mode in ("pos", "both"):
            self._log("开始定位测试: 仅开启 RMC 输出, 频度 %d 秒" % pos_freq, "info")
            try:
                frame = build_pwi_off()
                self._write(frame)
                self._log("设置: %s" % frame.decode("ascii", "replace").strip(), "tx")
                frame = build_ccrns_rmc()
                self._write(frame)
                self._log("设置: %s" % frame.decode("ascii", "replace").strip(), "tx")
                frame = build_rmc_freq(pos_freq)
                self._write(frame)
                self._log("设置: %s" % frame.decode("ascii", "replace").strip(), "tx")
            except Exception as exc:
                self._log("发送定位设置指令失败: %s" % exc, "err")
        self.status_var.set("测试进行中")

    def stop_test(self) -> None:
        if not self.running:
            return
        self.running = False
        if self.session is not None:
            for level, msg in self.session.stop():
                self._log(msg, level)
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        self.addr_entry.configure(state="normal")
        self.data_entry.configure(state="normal")
        self.interval_entry.configure(state="normal")
        self.pos_freq_entry.configure(state="normal")
        self.status_var.set("已停止")
        self._log("测试已停止", "info")
        self._close_csv()
        self._refresh_stats()

    def _send_once(self) -> None:
        if self.session is None or self.ser is None:
            return
        rec, frame, events = self.session.start_tx(time.monotonic())
        rec.wall_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        for level, msg in events:
            self._log(msg, level)
        try:
            self._write(frame)
        except Exception as exc:
            self._log("发送失败: %s" % exc, "err")
            self.session._finish(rec, "fail", "串口写入失败: %s" % exc, time.monotonic())
            return
        self._log("发送 #%d %s" % (rec.seq, frame.decode("ascii").strip()), "tx")
        self._refresh_stats()

    def _write(self, frame: bytes) -> None:
        with self.write_lock:
            if self.ser is None:
                raise RuntimeError("串口未打开")
            self.ser.write(frame)

    # -- 事件循环 -----------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        self._drain_queue(now)

        if self.running:
            if self.session is not None:
                for level, msg in self.session.tick(now):
                    self._log(msg, level)
                if now >= self.next_send_at:
                    self._send_once()
                    interval = int(self.interval_var.get() or 65)
                    self.next_send_at = now + interval
            self._check_pos_alive(now)
            self._refresh_stats()

        if self.icr_deadline > 0 and now >= self.icr_deadline:
            self.icr_deadline = 0.0
            if not self.addr_var.get().strip():
                self.addr_state.configure(text="查询超时", foreground="#e02424")
                self._log("地址查询超时: 5 秒内未收到 ICP/IBD 输出", "warn")

        self.root.after(100, self._tick)

    def _check_pos_alive(self, now: float) -> None:
        """定位测试长时间收不到 RMC 时告警, 60 秒内最多一次."""
        if self.pos_session is None:
            return
        try:
            freq = int(self.pos_freq_var.get())
        except ValueError:
            freq = 1
        if now - self.pos_last_seen <= max(30.0, freq * 3.0):
            return
        if now - self.pos_warned_at < 60.0:
            return
        self.pos_warned_at = now
        self._log("已 %.0f 秒未收到 RMC 帧, 检查终端是否已开启定位输出"
                  % (now - self.pos_last_seen), "warn")

    def _drain_queue(self, now: float) -> None:
        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "line":
                self._on_line(payload, now)
            elif kind == "error":
                self._log("串口错误: %s" % payload, "err")
                self.status_var.set("串口错误")

    def _on_line(self, raw: bytes, now: float) -> None:
        sent = parse_sentence(raw)
        text = raw.decode("ascii", "replace")
        if sent is None:
            self._log("接收 %s (无法解析)" % text, "warn")
            return
        if not sent.checksum_ok:
            self._log("接收 %s (校验和错误, 已忽略)" % text, "warn")
            return
        if sent.type_id.upper() == "ACKOK":
            if not self.ack_shown:
                self.ack_shown = True
                self._log("设备确认: %s" % text.strip(), "ok")
            return
        self._log("接收 %s" % text, "rx")
        self._learn_address(sent)
        if sent.type_id.upper() == "BDRNX":
            self._on_bdrnx(sent)
        if self.session is not None:
            for level, msg in self.session.on_sentence(sent, now):
                self._log(msg, level)
        if self.running and self.pos_session is not None \
                and sent.type_id.upper().endswith("RMC"):
            #每收到一帧 RMC 就刷新存活时刻, 否则断流告警会误报
            self.pos_last_seen = now
            self.pos_warned_at = 0.0
            for level, msg in self.pos_session.on_rmc(sent, now):
                self._log(msg, level)
        self._refresh_stats()

    def _on_bdrnx(self, sent: Sentence) -> None:
        """CCRNS 回显: 第 5 个字段为 1 表示 RMC 已打开."""
        if sent.field(5) == "1":
            self._log("设备回显: RMC 输出已打开", "ok")
        else:
            self._log("设备回显: RMC 未打开, 检查 $CCRNS 是否被接受", "warn")

    def _reset_stats(self) -> None:
        """清零两组统计显示, 开始新测试时调用."""
        for var in (self.var_sent, self.var_ok, self.var_fail, self.var_pending,
                    self.var_pos_total, self.var_pos_ok, self.var_pos_bad):
            var.set("0")
        self.var_rate.set("--")
        self.var_pos_rate.set("--")

    def _refresh_stats(self) -> None:
        if self.session is not None:
            self.var_sent.set(str(self.session.sent_count))
            self.var_ok.set(str(self.session.success_count))
            self.var_fail.set(str(self.session.fail_count))
            rate = self.session.success_rate
            self.var_rate.set("--" if rate is None else "%.1f%%" % rate)
            self.var_pending.set(str(self.session.pending_count))
        if self.pos_session is not None:
            self.var_pos_total.set(str(self.pos_session.total))
            self.var_pos_ok.set(str(self.pos_session.valid_count))
            self.var_pos_bad.set(str(self.pos_session.invalid_count))
            prate = self.pos_session.success_rate
            self.var_pos_rate.set("--" if prate is None else "%.1f%%" % prate)

    # -- 记录文件 -----------------------------------------------------------
    def _open_csv(self) -> None:
        """开始测试时新建主记录与失败记录; 建不了只提示, 不影响测试."""
        self._close_csv()
        folder = os.path.join(_HERE, "records")  #统一放子目录, 不在程序根目录散落文件
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as exc:
            self._log("创建记录目录失败: %s" % exc, "err")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = "test_record_%s" % stamp
        n = 1
        while os.path.exists(os.path.join(folder, base + ".csv")):
            n += 1
            base = "test_record_%s_%d" % (stamp, n)
        path = os.path.join(folder, base + ".csv")
        fail_path = os.path.join(folder, base.replace("test_record_", "fail_record_") + ".csv")
        fp = fail_fp = None
        writer = fail_writer = None
        try:
            fp = open(path, "w", newline="", encoding="utf-8-sig")
            writer = csv.writer(fp)
            writer.writerow(CSV_HEADER)
            fp.flush()
            fail_fp = open(fail_path, "w", newline="", encoding="utf-8-sig")
            fail_writer = csv.writer(fail_fp)
            fail_writer.writerow(CSV_HEADER)
            fail_fp.flush()
        except OSError as exc:
            for handle in (fp, fail_fp):
                if handle is not None:
                    try:
                        handle.close()
                    except OSError:
                        pass
            self._log("创建记录文件失败: %s" % exc, "err")
            return
        self.csv_fp, self.csv_writer, self.csv_path = fp, writer, path
        self.fail_fp, self.fail_writer = fail_fp, fail_writer
        self._log("记录文件: %s" % path, "info")
        self._log("失败记录: %s (只含失败条目)" % fail_path, "info")

    def _append_row(self, row: List, is_fail: bool) -> None:
        """主记录必写; 失败行同时追写到失败记录文件."""
        if self.csv_writer is None:
            return
        try:
            self.csv_writer.writerow(row)
            self.csv_fp.flush()
            if is_fail and self.fail_writer is not None:
                self.fail_writer.writerow(row)
                self.fail_fp.flush()
        except OSError as exc:
            if not self.csv_fail:
                self.csv_fail = True
                self._log("写记录文件失败: %s" % exc, "err")

    def _write_csv(self, rec: TxRecord, result: str, reason: str, now: float) -> None:
        """通信判定追写一行."""
        fki_text = "; ".join(
            "%s %s%s" % (t, s, "(%s)" % r if r not in ("", "0") else "")
            for t, s, r, _rem in rec.fki) or "未收到"
        if rec.tci_seen:
            tci_text = "已收到且匹配"
        elif rec.tci_mismatch:
            tci_text = "不匹配: %s" % rec.tci_mismatch
        else:
            tci_text = "未收到"
        row = [rec.seq, "通信", rec.wall_time, rec.addr, rec.data,
               self.interval_var.get(), RESULT_TEXT.get(result, result),
               "★" if result == "fail" else "", reason,
               "FKI: %s; 回环: %s" % (fki_text, tci_text),
               "%.1f" % max(0.0, now - rec.sent_at)]
        self._append_row(row, result == "fail")

    def _write_pos_csv(self, rec: PosRecord, valid: bool, reason: str,
                       now: float) -> None:
        """定位判定追写一行."""
        detail = "状态=%s 纬度=%s 经度=%s 模式=%s" % (
            rec.status or "空", rec.lat or "空", rec.lon or "空", rec.mode or "空")
        row = [rec.seq, "定位", rec.wall_time, "", "", self.pos_freq_var.get(),
               "成功" if valid else "失败", "" if valid else "★", reason,
               detail, ""]
        self._append_row(row, not valid)

    def _close_csv(self) -> None:
        """收尾关闭记录文件, 可重复调用."""
        for attr in ("csv_fp", "fail_fp"):
            handle = getattr(self, attr)
            if handle is not None:
                try:
                    handle.flush()
                    handle.close()
                except OSError:
                    pass
        self.csv_fp = None
        self.csv_writer = None
        self.fail_fp = None
        self.fail_writer = None

    # -- 日志 ---------------------------------------------------------------
    def _log(self, text: str, tag: str = "info") -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.configure(state="normal")
        self.log_text.insert("end", "[%s] %s\n" % (stamp, text), tag)
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 3000:
            self.log_text.delete("1.0", "501.0")
        self.log_text.configure(state="disabled")
        self.log_text.see("end")

    # -- 关闭 ---------------------------------------------------------------
    def on_close(self) -> None:
        if self.running:
            self.stop_test()
        self._close_csv()
        self.close_port()
        self.root.destroy()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="北斗三号终端测试工具")
    parser.add_argument("--simulate", action="store_true", help="启动即进入模拟模式")
    parser.add_argument("--port", default="", help="预填串口, 如 COM9")
    parser.add_argument("--baud", default="", help="预填波特率, 如 115200")
    parser.add_argument("--addr", default="", help="预填本机地址")
    parser.add_argument("--data", default="0123456789ABCDEF", help="预填测试数据")
    parser.add_argument("--interval", type=int, default=65, help="通信发送间隔(秒)")
    parser.add_argument("--mode", default="comm", choices=["comm", "pos", "both"],
                        help="测试模式: comm 仅通信, pos 仅定位, both 两者")
    parser.add_argument("--pos-freq", type=int, default=1, help="RMC 输出频度(秒)")
    parser.add_argument("--autostart", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    try:  # 高分屏下避免界面模糊
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    root = tk.Tk()
    app = App(root, simulate=args.simulate)
    if args.port:
        app.port_var.set(args.port)
    if args.baud:
        app.baud_var.set(args.baud)
    if args.addr:
        app.addr_var.set(args.addr)
    app.data_var.set(args.data)
    app.interval_var.set(str(args.interval))
    app.mode_var.set(args.mode)
    app.pos_freq_var.set(str(args.pos_freq))
    if args.autostart:  #自动化测试用: 界面就绪后直接开始
        root.after(800, app.start_test)
    root.mainloop()


if __name__ == "__main__":
    main()
