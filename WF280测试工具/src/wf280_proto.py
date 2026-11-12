# -*- coding: utf-8 -*-
"""WF280 全功能测试协议封装。

覆盖固件里 WF280 相关的全部命令：

    cmd[0] = 0x14                  异步整套测试（IsTaskDone 队列）  77 寄存器
    {1, 0x211}                     同步回读实际轨压                 状态 + mV
    {11}                           OTP 双 bank 校验                 结果 + 失败路号
    {1, 0x208, meas, settle}       输出测试 3.3 V                   状态 + 6 字节
    {1, 0x210, meas, settle}       输出测试 1.8 V                   状态 + 6 字节
    {1, 0x209, count}              读正常 bank 系数 (0x00..)        状态 + N 路
    {1, 0x207, count}              读 M1 bank 系数 + 电流 (0x80..)  状态 + 电流 + N 路
    {1, 0x201, mv_lo, mv_hi}       设定 DAC 轨压（辅助）            状态

整套测试（AllTestFor280）的结果帧和固件 Core/Inc/wf280.h 一一对应：

    [0]        状态：0=OK，非 0=错误码
    [1]        1.8 V 档实测轨压 mV（切 1.8 V 稳定后、读 M1 之前测）
    [2]        3.3 V 档实测轨压 mV（切 3.3 V 稳定后、读 NOR 之前测）
    [3..4]     工作电流原始码，低 16 位在前
    [5..6]     休眠电流原始码，低 16 位在前
    [7..12]    0xAC 读出的 6 字节（Status + 电桥 20 位 + 温度 20 位）
    [13..44]   M1 bank 32 路系数原始值
    [45..76]   NOR bank 32 路系数原始值

固件只回传电流的**原始码**，uA 值由本工具按同一套常数换算，
所以界面上的“解析电流”才是真实电流。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from mb_rtu import (
    HOLDING_BASE,
    INPUT_BASE,
    ModbusError,
    ModbusExceptionResponse,
    RtuMaster,
)

# ---------------- 命令码（与 CmdHelper.h 一致）----------------

CMD_SET_VDD = 0x201           # {1, 0x201, lo, hi}
CMD_GET_VERSION = 0x104       # {1, 0x104}    固件版本号
CMD_WF280_M1 = 0x207          # {1, 0x207, count}
CMD_WF280_OUTDATA = 0x208     # {1, 0x208, measurement, settle_ms}
CMD_WF280_COEFF = 0x209       # {1, 0x209, count}
CMD_WF280_2V_OUTDATA = 0x210  # {1, 0x210, measurement, settle_ms}（固件走 1.8 V）
CMD_WF280_RAIL = 0x211        # {1, 0x211}
CMD_GET_TASK_STATUS = 0x400
CMD_GET_ALLTEST_DATA = 0x401
CMD_WF280_OTP = 11            # {11}
CMD_WF280_FULL_TEST = 0x14    # {0x14}  触发 AllTestFor280

# ---------------- 固件常量 ----------------

RESULT_LENGTH = 77            # WF280_TEST_RESULT_LENGTH
OUTPUT_RESULT_LENGTH = 7      # WF280_OUTPUT_RESULT_LENGTH
COEFF_MAX = 32                # WF280_COEFF_MAX

# 整套测试结果帧偏移（与 wf280.h 的 WF280_R_* 一致）
R_STATUS = 0                  # 状态：0=OK，非 0=错误码
R_RAIL_1V8_MV = 1             # 1.8 V 档实测轨压 mV
R_RAIL_MV = 2                 # 3.3 V 档实测轨压 mV
R_WORK_RAW = 3                # 工作电流原始码，低 16 位在前
R_SLEEP_RAW = 5               # 休眠电流原始码，低 16 位在前
R_RAW = 7                     # 0xAC 读出的 6 字节原始数据，1 字节 1 寄存器
R_RAW_LEN = 6
R_M1 = 13                     # M1 bank 32 路系数原始值
R_NOR = 45                    # NOR bank 32 路系数原始值

RAIL_MIN_MV = 1500
RAIL_TOLERANCE_PCT = 10
TEST_RAIL_MV = 3300
# 这块板有两个档位：正常 bank 用 3.3 V，M1 bank 用 1.8 V，
# 回读判定两个都认，否则单独测 1.8 V 时会被误判成超差。
RAIL_LEVELS = ((3300, "3.3 V"), (1800, "1.8 V"))
SHORT_CURRENT_UA = 20000
# 固件里 3.3 V 用 20 ms、1.8 V 用 200 ms，但那是整机测试里轨已经稳了的前提。
# 单独发一条输出测试命令时轨是冷的，实测 20 ms 会报「NOR 忙等超时」，
# 所以这里默认给宽一点，需要严格对标固件时用 --settle 传 20/200。
SETTLE_3V3_MS = 300
SETTLE_1V8_MS = 400

STATUS_TEXT = {
    0x00: "OK",
    0x01: "I2C 通讯失败",
    0x02: "NOR 忙等超时",
    0x03: "命令响应超时",
    0xE1: "电流短路",
    0xE2: "轨压异常",
    0xE3: "M1 / NOR 系数不一致",
    0xF1: "设备忙（测试进行中）",
    0xF2: "命令长度错误",
}


def status_text(code: int) -> str:
    return STATUS_TEXT.get(code, f"未定义状态 0x{code:04X}")


def rail_in_level(mv: int, level: int) -> bool:
    """mv 是否落在 level 档位 ±RAIL_TOLERANCE_PCT 的窗口内。"""
    low = level * (100 - RAIL_TOLERANCE_PCT) // 100
    high = level * (100 + RAIL_TOLERANCE_PCT) // 100
    return low <= mv <= high


# ---------------- 电流原始码换算 ----------------

# 与固件 wf280.c 同一套常数
ADC_FULL_SCALE = 8388608        # WF280_ADC_FULL_SCALE，24 位满量程
CURRENT_SHUNT_OHM = 30          # WF280_CURRENT_SHUNT_OHM（板上实测分流电阻）


def signed24(raw: int) -> int:
    """把 24 位补码解释成有符号整数。

    ADS1220 是 24 位 ADC，负数在固件里被符号扩展成了 32 位，
    所以 0xFFFD29AE 实际是 0xFD29AE = -185938，不是 42 亿。
    """
    value = raw & 0xFFFFFF
    return value - (1 << 24) if value & 0x800000 else value


def signed20(raw: int) -> int:
    """把 20 位补码解释成有符号整数（WF280 电桥/温度数据是 20 位）。"""
    value = raw & 0xFFFFF
    return value - (1 << 20) if value & 0x80000 else value


def unpack_bt(raw: list[int], offset: int) -> tuple[int, int]:
    """按手册 4.3 拆开 0xAC 的 6 字节读出（不含 Status，从 d1 起 5 字节）。

    d1=BridgeDat[19:12]、d2=BridgeDat[11:4]、d3=BridgeDat[3:0]+TempDat[19:16]
    共用字节、d4=TempDat[15:8]、d5=TempDat[7:0]。返回 (bridge20, temp20)。
    """
    d1, d2, d3, d4, d5 = (raw[offset + i] & 0xFF for i in range(5))
    bridge = (d1 << 12) | (d2 << 4) | (d3 >> 4)
    temp = ((d3 & 0x0F) << 16) | (d4 << 8) | d5
    return bridge, temp


def signed16(value: int) -> int:
    """16 位补码转有符号（M1 结果只回传低 16 位）。"""
    value &= 0xFFFF
    return value - (1 << 16) if value & 0x8000 else value


def current_code(raw32: int) -> int:
    """结果帧里的 32 位原始码还原成 ADS1220 的 24 位有符号值。"""
    return signed24(raw32 & 0xFFFFFF)


def raw_to_ua(raw32: int, rail_mv: int) -> float:
    """按固件同一公式把原始码换成 uA。

    固件把 24 位码符号扩展后按 32 位回传，高 16 位是 0xFFFF 只是符号扩展，
    真正有效的是低 24 位。这颗板子的电流通道读出来是负的（分流方向相反），
    负号只代表电流方向，所以取绝对值再换算。
    """
    if not rail_mv:
        return 0.0
    return (abs(current_code(raw32)) * rail_mv * 1000
            / (ADC_FULL_SCALE * CURRENT_SHUNT_OHM))


class Result:
    """所有测试结果的共同接口：ok 与 rows()，界面和命令行共用。"""

    @property
    def ok(self) -> bool:  # pragma: no cover - 由子类实现
        raise NotImplementedError

    def rows(self) -> list[tuple[str, str, str]]:  # pragma: no cover
        raise NotImplementedError


# ---------------- ⓪ 固件版本 ----------------


@dataclass
class VersionResult(Result):
    """{1, 0x104}：FIRMWARE_VERSION 两字节，高字节是主版本。"""

    major: int
    minor: int

    @property
    def ok(self) -> bool:
        # 全 0 基本可以断定设备没真正应答
        return not (self.major == 0 and self.minor == 0)

    @property
    def text(self) -> str:
        return f"v{self.major}.{self.minor}"

    def rows(self):
        return [("固件版本", self.text, "OK" if self.ok else "NG"),
                ("原始值", f"0x{self.major:02X} 0x{self.minor:02X}", "—")]


# ---------------- ① 整套测试 ----------------


@dataclass
class FullTestResult(Result):
    """AllTestFor280 的 77 个结果寄存器。

    首字是状态，成功时后面依次是 1.8 V / 3.3 V 两档实测轨压、工作/休眠电流原始码、
    0xAC 的 6 字节原始数据，最后是 M1 / NOR 两个 bank 各 32 路原始值。
    失败时固件只写首字错误码，[1..76] 全 0。
    """

    regs: list[int] = field(default_factory=list)

    def __post_init__(self):
        regs = [int(v) & 0xFFFF for v in self.regs[:RESULT_LENGTH]]
        if len(regs) < RESULT_LENGTH:
            regs += [0] * (RESULT_LENGTH - len(regs))
        self.regs = regs

    status = property(lambda self: self.regs[R_STATUS])
    rail_1v8_mv = property(lambda self: self.regs[R_RAIL_1V8_MV])
    rail_mv = property(lambda self: self.regs[R_RAIL_MV])
    work_raw = property(lambda self: self.regs[R_WORK_RAW]
                        | (self.regs[R_WORK_RAW + 1] << 16))
    sleep_raw = property(lambda self: self.regs[R_SLEEP_RAW]
                         | (self.regs[R_SLEEP_RAW + 1] << 16))
    raw = property(lambda self: [self.regs[R_RAW + i] & 0xFF
                                 for i in range(R_RAW_LEN)])
    m1 = property(lambda self: list(self.regs[R_M1:R_M1 + COEFF_MAX]))
    nor = property(lambda self: list(self.regs[R_NOR:R_NOR + COEFF_MAX]))

    @property
    def status_ok(self) -> bool:
        """首字为 0：固件自检（含 M1/NOR 逐路比对）通过。"""
        return self.status == 0

    @property
    def work_code(self) -> int:
        """工作电流原始码（24 位有符号）。"""
        return current_code(self.work_raw)

    @property
    def sleep_code(self) -> int:
        """休眠电流原始码（24 位有符号）。"""
        return current_code(self.sleep_raw)

    @property
    def work_ua(self) -> float:
        return raw_to_ua(self.work_raw, self.rail_mv)

    @property
    def sleep_ua(self) -> float:
        return raw_to_ua(self.sleep_raw, self.rail_mv)

    @property
    def chip_status(self) -> int:
        return self.raw[0]

    @property
    def bridge_code(self) -> int:
        return unpack_bt(self.raw, 1)[0]

    @property
    def temp_code(self) -> int:
        return unpack_bt(self.raw, 1)[1]

    @property
    def coeff_available(self) -> bool:
        """失败时固件不追加系数，32 路全 0 不可信。"""
        return self.status_ok

    def coeff_pairs(self) -> list[tuple[int, int | None, int | None, bool]]:
        """32 路 (路号, M1, NOR, 是否一致)；失败时值为 None。"""
        if not self.coeff_available:
            return [(i, None, None, False) for i in range(COEFF_MAX)]
        m1, nor = self.m1, self.nor
        return [(i, m1[i], nor[i], m1[i] == nor[i]) for i in range(COEFF_MAX)]

    @property
    def coeff_mismatch(self) -> int | None:
        """第一个不一致的路号（0 基），全部一致返回 None。"""
        for index, _, _, same in self.coeff_pairs():
            if not same:
                return index
        return None

    @property
    def rail_1v8_ok(self) -> bool:
        """1.8 V 档轨压是否落在 1800 mV ±10%。

        只用于界面展示这一档的电压是否正常；总判定仍然看 3.3 V 档，
        这样老产线的 PASS/NG 规则不变。
        """
        return rail_in_level(self.rail_1v8_mv, 1800)

    @property
    def ok(self) -> bool:
        """首字为 0、轨压真的建立、32 路系数逐路一致，才算这块板通过。"""
        return (self.status_ok and self.rail_mv >= RAIL_MIN_MV
                and self.coeff_mismatch is None)

    @property
    def all_ok(self) -> bool:
        return self.ok

    def verdict_text(self) -> str:
        """一句话结论，给界面的大字和日志用。"""
        if not self.status_ok:
            return f"NG · {status_text(self.status)}"
        if self.rail_mv < RAIL_MIN_MV:
            return f"NG · 轨压未建立（{self.rail_mv} mV）"
        if self.coeff_mismatch is not None:
            return f"NG · 第 {self.coeff_mismatch + 1} 路系数不一致"
        if self.work_ua >= SHORT_CURRENT_UA or self.sleep_ua >= SHORT_CURRENT_UA:
            return "NG · 疑似短路"
        return "PASS"

    def _current_verdict(self, ua: float) -> str:
        if not self.rail_mv:
            return "—（无轨压，无法换算）"
        return "NG 疑似短路" if ua >= SHORT_CURRENT_UA else "OK"

    def _fail_rows(self) -> list[tuple[str, str, str]]:
        return [
            ("测量整体", "NG", "NG"),
            ("状态字 [0]", f"0x{self.status:04X}（{status_text(self.status)}）",
             "NG"),
            ("失败说明",
             "固件在失败点立即返回，[1..76] 全 0，轨压 / 电流 / 原始数据 / "
             "系数都没有测", "—"),
            ("建议",
             "先重测一次；仍失败就用「读 M1」「读系数」逐路比对定位",
             "—"),
        ]

    def _bt_rows(self) -> list[tuple[str, str, str]]:
        if not self.status_ok:
            return []
        return [
            ("0xAC 原始数据（6 字节）",
             " ".join(f"{b:02X}" for b in self.raw), "—"),
            ("电桥原始码（20 位）",
             f"0x{self.bridge_code:05X}（有符号 {signed20(self.bridge_code)}）",
             "—"),
            ("温度原始码（20 位）",
             f"0x{self.temp_code:05X}（有符号 {signed20(self.temp_code)}）",
             "—"),
        ]

    def _coeff_row(self) -> tuple[str, str, str]:
        if not self.coeff_available:
            return ("M1 / NOR 系数", "—", "—（失败帧不带系数）")
        mismatch = self.coeff_mismatch
        if mismatch is None:
            return ("M1 / NOR 系数", f"32 路全部一致",
                    f"OK（0x{self.m1[0]:04X} … 0x{self.m1[-1]:04X}）")
        return ("M1 / NOR 系数",
                f"第 {mismatch + 1} 路不一致：M1 0x{self.m1[mismatch]:04X} / "
                f"NOR 0x{self.nor[mismatch]:04X}", "NG")

    def rows(self):
        """界面表格的行，顺序就是产线要看的顺序。"""
        if not self.status_ok:
            return self._fail_rows()

        rail_ok = self.rail_mv >= RAIL_MIN_MV
        rows: list[tuple[str, str, str]] = [
            ("测量整体", "PASS", "OK"),
            ("状态字 [0]", "0x0000（OK）", "—"),
            ("轨压 1.8 V（实测）", f"{self.rail_1v8_mv} mV",
             "OK" if self.rail_1v8_ok else "超差（参考）"),
            ("轨压 3.3 V（实测）", f"{self.rail_mv} mV",
             "OK" if rail_ok else "NG 轨未建立"),
            ("工作电流原始码",
             f"0x{self.work_raw:08X}（{self.work_code}）", "—"),
            ("工作电流（解析）", f"{self.work_ua:.2f} uA",
             self._current_verdict(self.work_ua)),
            ("休眠电流原始码",
             f"0x{self.sleep_raw:08X}（{self.sleep_code}）", "—"),
            ("休眠电流（解析）", f"{self.sleep_ua:.2f} uA",
             self._current_verdict(self.sleep_ua)),
        ]
        rows.extend(self._bt_rows())
        rows.append(self._coeff_row())
        return rows


# ---------------- ② 轨压回读 ----------------


@dataclass
class RailReading(Result):
    status: int
    mv: int

    @property
    def ok(self) -> bool:
        return self.status == 0 and self.mv > 0

    @property
    def verdict(self) -> str:
        if self.status != 0:
            return status_text(self.status)
        if self.mv < RAIL_MIN_MV:
            return f"轨未建立（{self.mv} mV < {RAIL_MIN_MV} mV）"
        for level, name in RAIL_LEVELS:
            low = level * (100 - RAIL_TOLERANCE_PCT) // 100
            high = level * (100 + RAIL_TOLERANCE_PCT) // 100
            if low <= self.mv <= high:
                return f"正常（{name} 档）"
        ranges = "、".join(f"{lv}±{RAIL_TOLERANCE_PCT}%"
                           for lv, _ in RAIL_LEVELS)
        return f"不在正常档位（{ranges} mV）"

    def rows(self):
        return [("轨压（实测）", f"{self.mv} mV", self.verdict),
                ("状态", status_text(self.status), "OK" if self.status == 0 else "NG")]


# ---------------- ③ OTP 校验 ----------------


@dataclass
class OtpResult(Result):
    """{11}：两个 bank 逐路比对，out[0]=1 表示通过，0xFF 表示失败。"""

    status: int
    fail_index: int = 0

    @property
    def ok(self) -> bool:
        return self.status == 1

    @property
    def verdict(self) -> str:
        if self.ok:
            return "通过"
        if self.status == 0xFF:
            return f"第 {self.fail_index} 路不一致" if self.fail_index else "校验失败"
        return status_text(self.status)

    def rows(self):
        return [("OTP 校验", "通过" if self.ok else "不一致", "OK" if self.ok else "NG"),
                ("失败路号", str(self.fail_index), "—")]


# ---------------- ④⑤ 输出测试 3.3 V / 1.8 V ----------------


@dataclass
class OutputResult(Result):
    """{1, 0x208/0x210, measurement, settle}：状态 + 6 个原始字节。"""

    supply_mv: int
    status: int
    data: list[int] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == 0

    def rows(self):
        return [
            ("供电", f"{self.supply_mv} mV", "—"),
            ("状态", status_text(self.status), "OK" if self.ok else "NG"),
            ("0xAC 数据", " ".join(f"{b:02X}" for b in self.data), "—"),
            ("数据（十进制）", " ".join(str(b) for b in self.data), "—"),
        ]


# ---------------- ⑥ 读系数 ----------------


@dataclass
class CoeffResult(Result):
    """{1, 0x209, count}：正常 bank（0x00 起）的 N 路系数。"""

    start: int
    status: int
    values: list[int] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == 0

    def rows(self):
        rows = [("状态", status_text(self.status), "OK" if self.ok else "NG")]
        for i, value in enumerate(self.values):
            rows.append((f"系数 0x{self.start + i:02X}", f"0x{value:04X}", "—"))
        return rows


# ---------------- ⑦ 读 M1 ----------------


@dataclass
class M1Result(Result):
    """{1, 0x207, count}：1.8 V 下的待机/工作电流原始码 + M1 系数。"""

    status: int
    standby_code: int = 0
    work_code: int = 0
    values: list[int] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == 0

    def rows(self):
        rows = [
            ("状态", status_text(self.status), "OK" if self.ok else "NG"),
            ("待机电流码", f"0x{self.standby_code:04X}"
             f"（有符号 {signed16(self.standby_code)}）", "—"),
            ("工作电流码", f"0x{self.work_code:04X}"
             f"（有符号 {signed16(self.work_code)}）", "—"),
            ("提示", "固件只回传低 16 位原始码，电流值无法准确换算", "—"),
        ]
        for i, value in enumerate(self.values):
            rows.append((f"M1 系数 0x{0x80 + i:02X}", f"0x{value:04X}", "—"))
        return rows


# ---------------- 命令交互 ----------------


class Wf280Tester:
    """把固件的命令-结果约定包装成直观的方法。"""

    def __init__(self, master: RtuMaster, *, cmd_addr: int = HOLDING_BASE,
                 in_addr: int = INPUT_BASE):
        self.master = master
        self.cmd_addr = cmd_addr
        self.in_addr = in_addr

    # 不同固件构建对地址 0 基/1 基的处理不一样，连上后先试一遍。
    BASE_CANDIDATES = ((HOLDING_BASE, INPUT_BASE), (40001, 30001))

    def detect_bases(self) -> tuple[int, int]:
        """试出这块固件认的寄存器基址，成功返回 (命令地址, 结果地址)。"""
        last: Exception | None = None
        for cmd_addr, in_addr in self.BASE_CANDIDATES:
            try:
                self.master.read_input_registers(in_addr, 1)
            except ModbusExceptionResponse as exc:
                last = exc          # 设备在，但这个地址不认，试下一个
                continue
            self.cmd_addr, self.in_addr = cmd_addr, in_addr
            return cmd_addr, in_addr
        raise ModbusError(f"寄存器基址都对不上，最后一次错误：{last}")

    # 写命令 → 读结果。固件在写保持寄存器的回调里同步执行 Parse()，
    # 结果放在 sendBuf 里，随后从输入寄存器读回来。
    def _command(self, values: list[int], read_count: int) -> list[int]:
        self.master.write_multiple_registers(self.cmd_addr, values)
        if read_count <= 0:
            return []
        return self.master.read_input_registers(self.in_addr, read_count)

    # ---- ⓪ 固件版本 ----
    def read_version(self) -> VersionResult:
        out = self._command([1, CMD_GET_VERSION], 2)
        return VersionResult(major=out[0], minor=out[1])

    # ---- ① 异步整套测试 ----
    def get_task_status(self) -> int:
        """1 = 已入队/正在跑，0 = 空闲/已完成。"""
        return self._command([1, CMD_GET_TASK_STATUS], 1)[0]

    def read_results(self, count: int = RESULT_LENGTH) -> FullTestResult:
        return FullTestResult(self._command([1, CMD_GET_ALLTEST_DATA], count))

    def start_full_test(self) -> int:
        """触发 cmd[0]=0x14，固件回 out[0]=1 表示已入队。"""
        return self._command([CMD_WF280_FULL_TEST], 1)[0]

    def run_full_test(self, timeout: float = 60.0, interval: float = 0.2,
                      on_tick=None) -> FullTestResult:
        """触发整套测试并等它跑完。

        固件自己会切 1.8 V 读 M1、切 3.3 V 读 NOR 并逐路比对，
        全部一致后再测轨压/工作电流/0xAC/休眠电流，实测约 2~3 秒。
        """
        ack = self.start_full_test()
        if ack != 1:
            raise RuntimeError(f"触发测试失败，设备返回 {status_text(ack)}")
        deadline = time.monotonic() + timeout
        start = time.monotonic()
        while True:
            if not self.get_task_status():
                break
            if on_tick:
                on_tick(time.monotonic() - start)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"整套测试超时（{timeout:.0f} s 未完成），检查 DUT 是否插好")
            time.sleep(interval)
        return self.read_results()

    def run_full_test_fixed(self, delay: float = 3.0) -> FullTestResult:
        """产线流程：写 [14] → 等 delay 秒 → 写 [1,0x401] → 读 77 个寄存器。

        不轮询 0x400，只靠固定延时；如果到点还没跑完，0x401 会回 0xF1，
        这里再补一次轮询，避免把“还在跑”误当成结果。
        """
        self.master.write_multiple_registers(
            self.cmd_addr, [CMD_WF280_FULL_TEST])
        if delay > 0:
            time.sleep(delay)
        result = self.read_results()
        if result.status == 0xF1:              # 固定延时不够，改用轮询收尾
            deadline = time.monotonic() + 60.0
            while self.get_task_status():
                if time.monotonic() >= deadline:
                    raise TimeoutError("整套测试超时（固定延时后仍在运行）")
                time.sleep(0.2)
            result = self.read_results()
        return result

    # ---- ② 轨压 ----
    def read_rail(self) -> RailReading:
        out = self._command([1, CMD_WF280_RAIL], 2)
        return RailReading(status=out[0], mv=out[1])

    # ---- ③ OTP ----
    def read_otp(self) -> OtpResult:
        out = self._command([CMD_WF280_OTP], 2)
        return OtpResult(status=out[0], fail_index=out[1])

    # ---- ④⑤ 输出测试 ----
    def test_output(self, supply_mv: int = TEST_RAIL_MV,
                    measurement: int = 0, settle_ms: int | None = None) -> OutputResult:
        if supply_mv not in (1800, 3300):
            raise ValueError("输出测试只支持 1800 mV 或 3300 mV")
        if settle_ms is None:
            settle_ms = SETTLE_1V8_MS if supply_mv == 1800 else SETTLE_3V3_MS
        cmd_head = (CMD_WF280_2V_OUTDATA if supply_mv == 1800
                    else CMD_WF280_OUTDATA)
        out = self._command(
            [1, cmd_head, measurement & 0xFFFF, settle_ms & 0xFFFF],
            OUTPUT_RESULT_LENGTH)
        return OutputResult(supply_mv=supply_mv, status=out[0], data=out[1:7])

    def test_output_3v3(self, measurement: int = 0,
                        settle_ms: int | None = None) -> OutputResult:
        return self.test_output(3300, measurement, settle_ms)

    def test_output_1v8(self, measurement: int = 0,
                        settle_ms: int | None = None) -> OutputResult:
        return self.test_output(1800, measurement, settle_ms)

    # ---- ⑥ 系数 ----
    def read_coeff(self, count: int = 4) -> CoeffResult:
        count = max(1, min(COEFF_MAX, int(count)))
        out = self._command([1, CMD_WF280_COEFF, count], count + 1)
        return CoeffResult(start=0, status=out[0], values=out[1:count + 1])

    # ---- ⑦ M1 ----
    def read_m1(self, count: int = 4) -> M1Result:
        count = max(1, min(COEFF_MAX, int(count)))
        out = self._command([1, CMD_WF280_M1, count], 5 + count)
        standby = (out[1] & 0xFF) | ((out[2] & 0xFF) << 8)
        working = (out[3] & 0xFF) | ((out[4] & 0xFF) << 8)
        return M1Result(status=out[0], standby_code=standby,
                        work_code=working, values=out[5:5 + count])

    # ---- 辅助：设定 DAC 轨压 ----
    def set_supply(self, mv: int) -> int:
        mv = max(0, min(5000, int(mv)))
        out = self._command([1, CMD_SET_VDD, mv & 0xFF, (mv >> 8) & 0xFF], 1)
        return out[0]
