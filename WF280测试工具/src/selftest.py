# -*- coding: utf-8 -*-
"""离线自检：不需要真板子，用假串口模拟固件行为。

逐条验证 WF280 的全部命令：
    Modbus 帧的 CRC / 功能码 / 寄存器地址（40000 写、30000 读）
    cmd 0x14 的异步队列流程（触发 → 轮询 IsTaskDone → 取 89 个寄存器）
    新结果帧的解码：状态、1.8 V / 3.3 V 两档实测轨压、工作/休眠电流原始码与解析值、
    0xAC 的 6 字节原始数据、M1 / NOR 两个 bank 的 32 路系数
    0x211 轨压、11 OTP、0x208/0x210 输出测试、0x209 系数、0x207 M1、0x201 设压

运行：python selftest.py
"""

from __future__ import annotations

import struct
import sys

import mb_rtu
from mb_rtu import RtuMaster
from wf280_proto import (
    COEFF_MAX,
    R_M1,
    R_NOR,
    R_RAIL_1V8_MV,
    R_RAIL_MV,
    R_RAW,
    R_RAW_LEN,
    R_RAW_SAMPLES,
    R_SLEEP_RAW,
    R_STATUS,
    R_WORK_RAW,
    RESULT_LENGTH,
    FullTestResult,
    Wf280Tester,
    raw_to_ua,
    signed20,
    signed24,
    status_text,
)

SLAVE = 1

# 真机读到的电流原始码：24 位补码 -185938，按 32 位符号扩展后是 0xFFFD29AE
WORK_RAW = 0xFFFD29AE
SLEEP_RAW = 0xFFFFF831


class FakeFirmwareSerial:
    """假串口：收到主站帧就按固件逻辑生成应答。"""

    def __init__(self, *, rail_mv=3298, rail_1v8_mv=1796, busy_polls=3,
                 stuck=False, otp_fail=0, version=(1, 0), instant=False,
                 status=0, holding_base=40000, input_base=30000):
        self.is_open = True
        self.rx = bytearray()
        self.tx_buffer = bytearray()
        self.rail_mv = rail_mv
        self.rail_1v8_mv = rail_1v8_mv
        self.busy_polls = busy_polls
        self.stuck = stuck
        self.otp_fail = otp_fail          # 0=通过，非 0=第几路失败
        self.version = version
        self.instant = instant            # 测完立即完成：IsTaskDone 马上是 0
        self.status = status              # 整套测试首字返回的错误码
        self.holding_base = holding_base  # 真板子实测是 40000/30000（0 基）
        self.input_base = input_base
        self.busy = False
        self.supply_mv = 0
        self.send_buf = [0] * 120
        self.done_regs = None             # instant 模式下场记的结果
        self.commands_seen: list[list[int]] = []

    # ---------- pyserial 接口 ----------
    def reset_input_buffer(self):
        self.rx.clear()

    def write(self, data):
        self.rx.extend(data)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def read(self, size):
        if not self.tx_buffer:
            self._handle_request()
        chunk = bytes(self.tx_buffer[:size])
        del self.tx_buffer[:size]
        return chunk

    # ---------- 假固件行为 ----------
    def _results(self) -> list[int]:
        regs = [0] * RESULT_LENGTH
        if self.status:
            # 固件在失败点立即返回：只有首字有错误码，[1..76] 全 0
            regs[R_STATUS] = self.status
            return regs

        regs[R_STATUS] = 0
        regs[R_RAIL_1V8_MV] = self.rail_1v8_mv
        regs[R_RAIL_MV] = self.rail_mv
        regs[R_WORK_RAW] = WORK_RAW & 0xFFFF          # 低 16 位在前
        regs[R_WORK_RAW + 1] = (WORK_RAW >> 16) & 0xFFFF
        regs[R_SLEEP_RAW] = SLEEP_RAW & 0xFFFF
        regs[R_SLEEP_RAW + 1] = (SLEEP_RAW >> 16) & 0xFFFF
        # 0xAC 六字节原始数据：d0=芯片 Status，
        # 按手册 4.3 打包：电桥 0x2ABCD（正）、温度 0xF1234（负，-0xEDCC）
        sample = [0x00, 0x2A, 0xBC, 0xDF, 0x12, 0x34]
        for s in range(R_RAW_SAMPLES):
            regs[R_RAW + s * R_RAW_LEN:R_RAW + (s + 1) * R_RAW_LEN] = sample
        for i in range(COEFF_MAX):
            regs[R_M1 + i] = 0x1000 + i
            regs[R_NOR + i] = 0x1000 + i
        return regs

    def _handle_request(self):
        if len(self.rx) < 8:
            return
        frame = bytes(self.rx)
        self.rx.clear()

        body, crc = frame[:-2], struct.unpack("<H", frame[-2:])[0]
        if mb_rtu.crc16(body) != crc:
            return                       # 固件直接丢掉 CRC 错的帧
        slave, fc = body[0], body[1]
        if slave != SLAVE:
            return

        if fc == 0x10:
            address, count, _ = struct.unpack(">HHB", body[2:7])
            if address != self.holding_base:
                # 地址不对就回异常码 02，跟真板子行为一致
                payload = struct.pack(">BB", 0x10 | 0x80, 0x02)
            else:
                values = list(struct.unpack(f">{count}H", body[7:]))
                self.commands_seen.append(values)
                self._run_command(values)
                payload = struct.pack(">BHH", 0x10, address, count)
        elif fc == 0x04:
            address, count = struct.unpack(">HH", body[2:6])
            if address != self.input_base:
                payload = struct.pack(">BB", 0x04 | 0x80, 0x02)
            else:
                data = b"".join(struct.pack(">H", v) for v in self.send_buf[:count])
                payload = struct.pack(">BB", 0x04, len(data)) + data
        else:
            payload = struct.pack(">BB", fc | 0x80, 0x01)

        self.tx_buffer.extend(mb_rtu.with_crc(bytes([SLAVE]) + payload))

    def _run_command(self, cmd: list[int]):
        # 对应 CmdHelper.c 的 Parse()
        if self.busy and not (cmd[0] == 1 and len(cmd) > 1
                              and cmd[1] in (0x400, 0x401)):
            self.send_buf = [0] * 120
            self.send_buf[0] = 0xF1
            return

        if cmd[0] == 0x14:                      # 异步整套测试
            self.send_buf = [0] * 120
            self.send_buf[0] = 1                # 固件先回 1，结果要等 0x401
            self.done_regs = None
            if self.instant:
                # 立刻测完：IsTaskDone 马上是 0，0x401 返回完整 89 个字
                self.busy = False
                self.done_regs = self._results()
            else:
                self.busy = True
                self.busy_polls = 3
            return

        if cmd[0] == 11:                      # OTP 校验
            self.send_buf = [0] * 120
            if self.otp_fail:
                self.send_buf[0] = 0xFF
                self.send_buf[1] = self.otp_fail
            else:
                self.send_buf[0] = 1
                self.send_buf[1] = 0
            return

        self.send_buf = [0] * 120
        if cmd[0] != 1 or len(cmd) < 2:
            return
        head = cmd[1]

        if head == 0x400:                     # 查询 IsTaskDone
            if self.stuck:
                self.send_buf[0] = 1
                return
            self.send_buf[0] = 1 if self.busy else 0
            if self.busy:
                self.busy_polls -= 1
                if self.busy_polls <= 0:
                    self.busy = False
            return

        if head == 0x401:                     # 取整套结果
            if self.busy:
                self.send_buf[0] = 0xF1
            else:
                regs = self.done_regs
                if regs is None:
                    regs = self._results()
                self.send_buf[:RESULT_LENGTH] = regs
            return

        if head == 0x211:                     # 同步读轨压
            if self.rail_mv:
                self.send_buf[0] = 0
                self.send_buf[1] = self.rail_mv
            else:
                self.send_buf[0] = 0xE2
            return

        if head == 0x104:                     # 固件版本
            self.send_buf[0] = self.version[0]
            self.send_buf[1] = self.version[1]
            return

        if head == 0x201:                     # 设定 DAC 轨压
            self.supply_mv = cmd[2] | (cmd[3] << 8)
            self.send_buf[0] = 0
            return

        if head in (0x208, 0x210):            # 输出测试
            self.send_buf[0] = 0
            for i, byte in enumerate(b"\xB1\xB2\xB3\xB4\xB5\xB6"):
                self.send_buf[1 + i] = byte
            return

        if head == 0x209:                     # 正常 bank 系数
            count = cmd[2]
            self.send_buf[0] = 0
            for i in range(count):
                self.send_buf[1 + i] = 0x0A00 + i
            return

        if head == 0x207:                     # M1
            count = cmd[2]
            self.send_buf[0] = 0
            self.send_buf[1] = 0x34           # 待机码低字节
            self.send_buf[2] = 0x12           # 待机码高字节
            self.send_buf[3] = 0x78           # 工作码低字节
            self.send_buf[4] = 0x56           # 工作码高字节
            for i in range(count):
                self.send_buf[5 + i] = 0x8000 + i
            return


def make_tester(**kwargs):
    fake = FakeFirmwareSerial(**kwargs)
    master = RtuMaster("FAKE")
    master._ser = fake                        # 跳过真实串口
    return fake, Wf280Tester(master)


CASES = []


def case(name):
    def deco(fn):
        CASES.append((name, fn))
        return fn
    return deco


@case("电流换算：负原始码按 24 位有符号解释")
def _():
    # 真机读到的原始码，固件按 32 位符号扩展后回传
    assert signed24(WORK_RAW) == -185938
    assert signed24(0xFFFD29AE & 0xFFFFFF) == -185938
    assert signed24(0x00000017) == 23
    ua = raw_to_ua(WORK_RAW, 3296)
    assert 2430 < ua < 2440, ua       # 约 2435.2 uA（分流 30 Ω）
    assert raw_to_ua(WORK_RAW, 0) == 0    # 没轨压不换算


@case("结果帧：89 个寄存器按固件偏移解码")
def _():
    regs = [0] * RESULT_LENGTH
    regs[R_STATUS] = 0
    regs[R_RAIL_1V8_MV] = 1796
    regs[R_RAIL_MV] = 3296
    regs[R_WORK_RAW] = WORK_RAW & 0xFFFF
    regs[R_WORK_RAW + 1] = WORK_RAW >> 16
    regs[R_SLEEP_RAW] = SLEEP_RAW & 0xFFFF
    regs[R_SLEEP_RAW + 1] = SLEEP_RAW >> 16
    regs[R_RAW:R_RAW + R_RAW_LEN] = [0x00, 0x2A, 0xBC, 0xDF, 0x12, 0x34]
    for i in range(COEFF_MAX):
        regs[R_M1 + i] = 0x1100 + i
        regs[R_NOR + i] = 0x2200 + i

    result = FullTestResult(regs)
    assert result.status == 0 and result.status_ok
    assert result.rail_1v8_mv == 1796 and result.rail_1v8_ok
    assert result.rail_mv == 3296
    assert result.work_raw == WORK_RAW
    assert result.sleep_raw == SLEEP_RAW
    assert result.work_code == -185938
    assert result.sleep_code == -1999
    assert 2430 < result.work_ua < 2440
    assert 26.1 < result.sleep_ua < 26.3, result.sleep_ua
    assert result.raw == [0x00, 0x2A, 0xBC, 0xDF, 0x12, 0x34]
    assert result.chip_status == 0x00
    assert result.bridge_code == 0x2ABCD
    assert signed20(result.bridge_code) == 0x2ABCD
    assert result.temp_code == 0xF1234
    assert signed20(result.temp_code) == -0xEDCC
    assert result.m1[0] == 0x1100 and result.m1[-1] == 0x111F
    assert result.nor[0] == 0x2200 and result.nor[-1] == 0x221F
    assert len(result.m1) == COEFF_MAX and len(result.nor) == COEFF_MAX
    assert len(result.regs) == RESULT_LENGTH
    # 这里两个 bank 故意给了不同的值：工具自己要能看出不一致
    assert not result.ok and result.coeff_mismatch == 0


@case("结果帧：显示顺序是 整体 → 两档轨压 → 电流 → 原始数据 → 系数")
def _():
    result = FullTestResult([0] * RESULT_LENGTH)
    result.regs[R_RAIL_1V8_MV] = 1796
    result.regs[R_RAIL_MV] = 3296
    result.regs[R_WORK_RAW] = WORK_RAW & 0xFFFF
    result.regs[R_WORK_RAW + 1] = WORK_RAW >> 16
    result.regs[R_SLEEP_RAW] = SLEEP_RAW & 0xFFFF
    result.regs[R_SLEEP_RAW + 1] = SLEEP_RAW >> 16
    result.regs[R_RAW:R_RAW + R_RAW_LEN] = [0x00, 0x2A, 0xBC, 0xDF, 0x12, 0x34]
    for i in range(COEFF_MAX):
        result.regs[R_M1 + i] = 0x1000 + i
        result.regs[R_NOR + i] = 0x1000 + i

    rows = result.rows()
    names = [row[0] for row in rows]
    assert names[0] == "测量整体", names
    assert names[1] == "状态字 [0]", names
    assert names[2] == "轨压 1.8 V（实测）", names
    assert names[3] == "轨压 3.3 V（实测）", names
    assert names[4] == "工作电流原始码", names
    assert names[5] == "工作电流（解析）", names
    assert names[6] == "休眠电流原始码", names
    assert names[7] == "休眠电流（解析）", names
    assert names[8] == "第 1 次 0xAC 原始数据", names
    assert names[9] == "第 2 次 0xAC 原始数据", names
    assert names[10] == "第 3 次 0xAC 原始数据", names
    assert names[11] == "M1 / NOR 系数", names

    by_name = {row[0]: row for row in rows}
    assert by_name["测量整体"][1] == "PASS"
    assert by_name["轨压 1.8 V（实测）"][1] == "1796 mV"
    assert by_name["轨压 1.8 V（实测）"][2] == "OK"
    assert by_name["轨压 3.3 V（实测）"][1] == "3296 mV"
    assert "0xFFFD29AE" in by_name["工作电流原始码"][1]
    assert by_name["工作电流（解析）"][1].endswith("uA")
    assert by_name["工作电流（解析）"][2] == "OK"
    assert by_name["第 1 次 0xAC 原始数据"][1] == "00 2A BC DF 12 34"
    assert "32 路全部一致" in by_name["M1 / NOR 系数"][1]


@case("结果帧：系数不一致（0xE3）只报状态，不当成数据")
def _():
    result = FullTestResult([0] * RESULT_LENGTH)
    result.regs[R_STATUS] = 0xE3
    assert not result.status_ok and not result.ok
    assert status_text(0xE3) == "M1 / NOR 系数不一致"
    rows = {row[0]: row for row in result.rows()}
    assert rows["测量整体"][1] == "NG"
    assert "0x00E3" in rows["状态字 [0]"][1]
    assert "M1 / NOR 系数不一致" in rows["状态字 [0]"][1]
    assert not result.coeff_available
    assert all(m1 is None for _, m1, _, _ in result.coeff_pairs())
    assert result.verdict_text().startswith("NG")


@case("结果帧：系数逐路比较能定位到具体路号")
def _():
    result = FullTestResult([0] * RESULT_LENGTH)
    result.regs[R_STATUS] = 0
    result.regs[R_RAIL_MV] = 3300
    for i in range(COEFF_MAX):
        result.regs[R_M1 + i] = 0x1234
        result.regs[R_NOR + i] = 0x1234
    result.regs[R_NOR + 5] = 0x1235

    pairs = result.coeff_pairs()
    assert len(pairs) == COEFF_MAX
    assert pairs[5] == (5, 0x1234, 0x1235, False)
    assert result.coeff_mismatch == 5
    assert "第 6 路" in result.verdict_text()
    row = {r[0]: r for r in result.rows()}["M1 / NOR 系数"]
    assert "第 6 路" in row[1] and row[2] == "NG"


@case("结果帧：轨压为 0 → NG 轨未建立")
def _():
    result = FullTestResult([0] * RESULT_LENGTH)
    result.regs[R_RAIL_MV] = 0
    assert result.status_ok and not result.ok
    rows = {row[0]: row for row in result.rows()}
    assert "NG" in rows["轨压 3.3 V（实测）"][2], rows
    assert "轨压未建立" in result.verdict_text()


@case("结果帧：电流超过 20 mA → 疑似短路")
def _():
    result = FullTestResult([0] * RESULT_LENGTH)
    result.regs[R_RAIL_MV] = 3300
    # 24 位满量程附近的原始码，换算后约 27 mA，超过 20 mA 阈值
    result.regs[R_WORK_RAW] = 0x7FFFFF & 0xFFFF
    result.regs[R_WORK_RAW + 1] = 0x007F
    assert result.work_ua > 20000
    rows = {row[0]: row for row in result.rows()}
    assert rows["工作电流（解析）"][2] == "NG 疑似短路"
    assert "疑似短路" in result.verdict_text()


@case("CRC16 标准向量")
def _():
    # “01 03 00 00 00 01” 的 Modbus CRC = 0x0A84，线上低字节在前 84 0A
    assert mb_rtu.crc16(bytes([1, 3, 0, 0, 0, 1])) == 0x0A84
    assert mb_rtu.with_crc(bytes([1, 3, 0, 0, 0, 1]))[-2:] == b"\x84\x0a"


@case("寄存器基址：识别 0 基（40000/30000，真板子就是这种）")
def _():
    _, tester = make_tester()
    assert tester.detect_bases() == (40000, 30000)


@case("寄存器基址：兼容 1 基固件（40001/30001）")
def _():
    fake, tester = make_tester(holding_base=40001, input_base=30001)
    assert tester.detect_bases() == (40001, 30001)
    result = tester.read_version()
    assert result.ok and fake.commands_seen[-1] == [1, 0x104]


@case("0x104 固件版本：读得到才算连对设备")
def _():
    fake, tester = make_tester(version=(1, 0))
    result = tester.read_version()
    assert fake.commands_seen[-1] == [1, 0x104]
    assert result.text == "v1.0" and result.ok

    # 全 0 视为设备没正常应答
    _, blank = make_tester(version=(0, 0))
    assert not blank.read_version().ok


@case("0x211 轨压：状态 + mV")
def _():
    fake, tester = make_tester(rail_mv=3298)
    reading = tester.read_rail()
    assert (reading.status, reading.mv) == (0, 3298)
    assert reading.ok and "3.3 V" in reading.verdict, reading.verdict
    assert fake.commands_seen[-1] == [1, 0x211]


@case("0x211 轨压：1.8 V 档同样判为正常")
def _():
    _, tester = make_tester(rail_mv=1795)
    reading = tester.read_rail()
    assert reading.ok and "1.8 V" in reading.verdict, reading.verdict


@case("0x211 轨压：轨没起来")
def _():
    _, tester = make_tester(rail_mv=0)
    reading = tester.read_rail()
    assert reading.status == 0xE2 and not reading.ok
    assert status_text(0xE2) == "轨压异常"


@case("0x14 整套测试：异步流程与 89 字结果解码")
def _():
    fake, tester = make_tester(rail_mv=3298)
    ticks: list[float] = []
    result = tester.run_full_test(interval=0.0, on_tick=ticks.append)

    assert result.ok and result.all_ok
    assert result.rail_1v8_mv == 1796
    assert result.rail_mv == 3298
    assert result.work_raw == WORK_RAW
    assert result.sleep_raw == SLEEP_RAW
    assert result.raw == [0x00, 0x2A, 0xBC, 0xDF, 0x12, 0x34]
    assert result.m1[0] == 0x1000 and result.nor[0] == 0x1000
    assert result.coeff_mismatch is None
    assert len(result.regs) == RESULT_LENGTH
    assert ticks, "应当至少有一次进度回调"
    assert fake.commands_seen[0] == [0x14]
    assert [1, 0x400] in fake.commands_seen
    assert fake.commands_seen[-1] == [1, 0x401]


@case("0x14 整套测试：首字错误码 1（I2C 失败）→ NG")
def _():
    _, tester = make_tester(status=0x01)
    result = tester.run_full_test(interval=0.0)
    assert not result.ok and not result.status_ok
    rows = {row[0]: row for row in result.rows()}
    assert "I2C" in rows["状态字 [0]"][1]


@case("0x14 整套测试：0xE3 系数不一致 → NG，且不显示假数据")
def _():
    _, tester = make_tester(status=0xE3)
    result = tester.run_full_test(interval=0.0)
    assert not result.ok
    assert result.rail_mv == 0 and result.m1 == [0] * COEFF_MAX
    assert not result.coeff_available
    assert result.verdict_text() == "NG · M1 / NOR 系数不一致"


@case("0x14 整套测试：超时保护")
def _():
    _, tester = make_tester(stuck=True)
    try:
        tester.run_full_test(timeout=0.3, interval=0.05)
    except TimeoutError as exc:
        assert "超时" in str(exc)
    else:
        raise AssertionError("应当在超时后抛异常")


@case("0x14（固定 3 s 版）：延时不够时自动补轮询")
def _():
    fake, tester = make_tester(rail_mv=3298)   # 假固件要 3 次轮询才算完成
    result = tester.run_full_test_fixed(delay=0.0)
    assert result.status == 0 and result.ok, result.status
    assert result.rail_mv == 3298
    # 先写 0x14，再取数拿到 0xF1，然后补轮询
    assert fake.commands_seen[0] == [0x14]
    assert [1, 0x401] in fake.commands_seen
    assert [1, 0x400] in fake.commands_seen


@case("0x14（固定 3 s 版）：测完直接取数")
def _():
    fake, tester = make_tester(rail_mv=3298, instant=True)
    result = tester.run_full_test_fixed(delay=0.0)
    assert result.ok and result.rail_mv == 3298
    assert fake.commands_seen == [[0x14], [1, 0x401]], fake.commands_seen


@case("11 OTP 校验：通过 / 不一致")
def _():
    fake, tester = make_tester()
    ok_result = tester.read_otp()
    assert ok_result.ok and ok_result.verdict == "通过"
    assert fake.commands_seen[-1] == [11]

    fake.otp_fail = 7
    bad = tester.read_otp()
    assert not bad.ok and bad.fail_index == 7
    assert "7" in bad.verdict


@case("0x208 输出测试 3.3 V：命令与数据")
def _():
    fake, tester = make_tester()
    result = tester.test_output_3v3(measurement=0x0003, settle_ms=120)
    assert fake.commands_seen[-1] == [1, 0x208, 0x0003, 120]
    assert result.ok and result.supply_mv == 3300
    assert result.data == [0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6]


@case("0x210 输出测试 1.8 V：用另一条命令头")
def _():
    fake, tester = make_tester()
    result = tester.test_output_1v8()
    assert fake.commands_seen[-1] == [1, 0x210, 0x0000, 400]
    assert result.ok and result.supply_mv == 1800


@case("输出测试：非法供电电压被拦下")
def _():
    _, tester = make_tester()
    for mv in (2500, 2000):
        try:
            tester.test_output(mv)
        except ValueError as exc:
            assert "1800" in str(exc) or "3300" in str(exc)
        else:
            raise AssertionError(f"{mv} mV 应当被拒绝")


@case("0x209 读系数：路数上限与解码")
def _():
    fake, tester = make_tester()
    result = tester.read_coeff(4)
    assert fake.commands_seen[-1] == [1, 0x209, 4]
    assert result.ok and result.values == [0x0A00, 0x0A01, 0x0A02, 0x0A03]

    tester.read_coeff(999)                 # 超上限应被夹到 COEFF_MAX
    assert fake.commands_seen[-1] == [1, 0x209, COEFF_MAX]

    # 界面上的「3.3 V 读 NOR」按钮：固定 32 路一帧
    full = tester.read_coeff(COEFF_MAX)
    assert fake.commands_seen[-1] == [1, 0x209, 32]
    assert full.ok and len(full.values) == 32


@case("0x207 读 M1：电流码与系数")
def _():
    fake, tester = make_tester()
    result = tester.read_m1(3)
    assert fake.commands_seen[-1] == [1, 0x207, 3]
    assert result.ok
    assert result.standby_code == 0x1234
    assert result.work_code == 0x5678
    assert result.values == [0x8000, 0x8001, 0x8002]

    # 界面上的「1.8 V 读 M1」按钮：固定 32 路一帧
    full = tester.read_m1(COEFF_MAX)
    assert fake.commands_seen[-1] == [1, 0x207, 32]
    assert full.ok and len(full.values) == 32


@case("0x201 设定轨压：低字节在前")
def _():
    fake, tester = make_tester()
    assert tester.set_supply(3300) == 0
    assert fake.commands_seen[-1] == [1, 0x201, 3300 & 0xFF, 3300 >> 8]
    assert fake.supply_mv == 3300


def main() -> int:
    failed = 0
    for name, fn in CASES:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"[ OK ] {name}")
    print(f"\n{len(CASES) - failed}/{len(CASES)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
