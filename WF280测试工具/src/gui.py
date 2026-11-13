# -*- coding: utf-8 -*-
"""图形界面（tkinter）。由 wf280_tool.py 在无参数时调用。

三块功能严格分开，互不串联：
    ① 整套测试   固件 cmd[0] = 0x14，即 AllTestFor280 那条综合测试
    ② 单项命令   OTP / 输出测试 / 读系数 / 读 M1 / 轨压回读 / 设定电压

整套测试的 89 个结果寄存器按产线关心的顺序展示：
    测量整体 → 实测轨压 → 工作电流（原始码 + 解析值）→ 休眠电流（原始码 +
    解析值）→ 0xAC 的 6 字节原始数据 → M1 / NOR 两个 bank 各 32 路系数。
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from mb_rtu import RtuMaster
from wf280_proto import (
    COEFF_MAX,
    FullTestResult,
    RailReading,
    Wf280Tester,
    status_text,
)

BAUD_RATES = ["115200", "57600", "38400", "19200", "9600"]
BG = "#f5f6f8"
CARD = "#ffffff"
TEXT = "#1f2328"
MUTED = "#6b7280"
ACCENT = "#1f6feb"
OK_COLOR = "#1a7f37"
NG_COLOR = "#cf222e"

APP_VERSION = "v1.3 · 2026-11-11"


class _Simple:
    """把只有状态码的命令包装成能进表格的结果。"""

    def __init__(self, status: int):
        self.status = status

    @property
    def ok(self) -> bool:
        return self.status == 0

    def rows(self):
        return [("状态", status_text(self.status), "OK" if self.ok else "NG")]


class TestApp:
    def __init__(self, root: "tk.Tk"):
        self.root = root
        self.master: RtuMaster | None = None
        self.tester: Wf280Tester | None = None
        self.busy = False
        self.version_text = "未知"
        self.device_info = ""
        self._queue: queue.Queue = queue.Queue()

        self._build_style()
        self._build_ui()
        self._refresh_ports()
        self._drain_queue()
        self._poll_auto_rail()

    # ---------- 样式 ----------

    def _build_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        face = ("Microsoft YaHei UI", 10)
        style.configure(".", font=face, background=BG, foreground=TEXT)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Card.TFrame", background=CARD)
        style.configure("Card.TLabel", background=CARD, foreground=TEXT)
        style.configure("Card.TCheckbutton", background=CARD, foreground=TEXT)
        style.configure("Muted.TLabel", background=CARD, foreground=MUTED,
                        font=("Microsoft YaHei UI", 9))
        style.configure("Head.TLabel", background=CARD, foreground=TEXT,
                        font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("TButton", padding=(10, 5))
        style.configure("Accent.TButton", padding=(14, 7),
                        font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(16, 7),
                        font=("Microsoft YaHei UI", 10))
        style.map("TNotebook.Tab",
                  background=[("selected", CARD)],
                  foreground=[("selected", ACCENT)])
        style.configure("Treeview", background=CARD, fieldbackground=CARD,
                        foreground=TEXT, rowheight=25, borderwidth=0)
        style.configure("Treeview.Heading",
                        font=("Microsoft YaHei UI", 9, "bold"),
                        background="#eceef1", foreground=TEXT)
        style.map("Treeview", background=[("selected", "#dbeafe")],
                  foreground=[("selected", TEXT)])

    # ---------- 布局 ----------

    def _build_ui(self) -> None:
        self.root.title(f"WF280 全功能测试工具 {APP_VERSION}")
        self.root.configure(bg=BG)
        # 产线电脑可能是 1366×768，窗口按屏幕大小收一点，别撑出屏幕外
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        self.root.geometry(f"{min(1060, screen_w - 60)}x{min(840, screen_h - 100)}")
        self.root.minsize(920, 640)

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        self._build_connection_bar(outer)
        self.info_label = ttk.Label(outer, text="未连接设备", foreground=MUTED)
        self.info_label.pack(fill="x", pady=(0, 8))

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True, pady=(0, 8))
        self.full_tab = ttk.Frame(self.notebook, style="Card.TFrame")
        self.item_tab = ttk.Frame(self.notebook, style="Card.TFrame")
        self.notebook.add(self.full_tab, text="① 整套测试（cmd 14）")
        self.notebook.add(self.item_tab, text="② 单项命令")

        self._build_full_tab(self.full_tab)
        self._build_item_tab(self.item_tab)
        self._build_log(outer)
        self._log(f"工具版本 {APP_VERSION}（窗口标题里也有，别开成旧的了）", "muted")

    def _build_connection_bar(self, parent) -> None:
        bar = ttk.Frame(parent)
        bar.pack(fill="x", pady=(0, 6))

        ttk.Label(bar, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=11,
                                     state="readonly")
        self.port_box.pack(side="left", padx=(4, 4))
        ttk.Button(bar, text="刷新", width=5,
                   command=self._refresh_ports).pack(side="left")

        ttk.Label(bar, text="波特率").pack(side="left", padx=(12, 4))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES,
                     width=7, state="readonly").pack(side="left")

        ttk.Label(bar, text="从机").pack(side="left", padx=(12, 4))
        self.slave_var = tk.StringVar(value="1")
        ttk.Spinbox(bar, from_=1, to=247, width=4,
                    textvariable=self.slave_var).pack(side="left")

        self.connect_btn = ttk.Button(bar, text="连接", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=(12, 6))
        self.version_btn = ttk.Button(bar, text="读版本", width=7,
                                      command=self._on_read_version, state="disabled")
        self.version_btn.pack(side="left")
        self.conn_label = ttk.Label(bar, text="未连接", foreground=MUTED)
        self.conn_label.pack(side="left", padx=(10, 0))

    def _tree(self, parent, height: int, columns) -> ttk.Treeview:
        tree = ttk.Treeview(parent, columns=[c[0] for c in columns],
                            show="headings", height=height)
        for key, text, width, anchor in columns:
            tree.heading(key, text=text)
            tree.column(key, width=width, anchor=anchor, stretch=True)
        tree.tag_configure("ok", foreground=OK_COLOR)
        tree.tag_configure("ng", foreground=NG_COLOR)
        return tree

    def _build_full_tab(self, parent) -> None:
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.pack(fill="both", expand=True)

        head = ttk.Frame(card, style="Card.TFrame")
        head.pack(fill="x")
        ttk.Label(head, text="AllTestFor280 · 89 个输入寄存器",
                  style="Head.TLabel").pack(side="left")
        ttk.Label(head,
                  text="固件自己切 1.8 V 读 M1、3.3 V 读 NOR 并逐路比对，"
                       "电压不用上位机设",
                  style="Muted.TLabel").pack(side="left", padx=(10, 0))

        row = ttk.Frame(card, style="Card.TFrame")
        row.pack(fill="x", pady=(8, 2))
        self.full_btn = ttk.Button(row, text="开始整套测试", style="Accent.TButton",
                                   command=self._on_full_test)
        self.full_btn.pack(side="left")
        self.full_delay_btn = ttk.Button(row, text="固定 3 s 后取数", width=16,
                                         command=self._on_full_test_delay)
        self.full_delay_btn.pack(side="left", padx=(8, 0))
        self.full_status = ttk.Label(row, text="待测", style="Card.TLabel")
        self.full_status.pack(side="left", padx=(14, 0))
        self.full_verdict = ttk.Label(row, text="", style="Card.TLabel",
                                      font=("Microsoft YaHei UI", 13, "bold"))
        self.full_verdict.pack(side="left", padx=(14, 0))

        ttk.Label(card,
                  text="流程：写 [0x14] → 轮询 [1,0x400] 等它跑完 → 写 [1,0x401] "
                       "读 89 个字（含 3 次 0xAC）。实测约 3~4 s。",
                  style="Muted.TLabel").pack(anchor="w", pady=(0, 8))

        body = ttk.Frame(card, style="Card.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        # 左：结果帧解析
        left = ttk.Frame(body, style="Card.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        ttk.Label(left, text="测量结果（按帧顺序）",
                  style="Card.TLabel").grid(row=0, column=0, sticky="w")
        wrap = ttk.Frame(left, style="Card.TFrame")
        wrap.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        self.full_tree = self._tree(wrap, 12, (
            ("item", "项目", 150, "w"),
            ("value", "数值", 260, "w"),
            ("verdict", "判定", 120, "w"),
        ))
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.full_tree.yview)
        self.full_tree.configure(yscrollcommand=sb.set)
        self.full_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        # 右：M1 / NOR 系数
        right = ttk.Frame(body, style="Card.TFrame")
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)
        rhead = ttk.Frame(right, style="Card.TFrame")
        rhead.grid(row=0, column=0, sticky="ew")
        ttk.Label(rhead, text="M1 / NOR 系数（32 路原始值）",
                  style="Card.TLabel").pack(side="left")
        self.coeff_label = ttk.Label(rhead, text="未测", style="Muted.TLabel")
        self.coeff_label.pack(side="right")
        wrap2 = ttk.Frame(right, style="Card.TFrame")
        wrap2.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        self.coeff_tree = self._tree(wrap2, 12, (
            ("ch", "路号", 56, "center"),
            ("m1", "M1", 82, "center"),
            ("nor", "NOR", 82, "center"),
            ("cmp", "比较", 74, "center"),
        ))
        sb2 = ttk.Scrollbar(wrap2, orient="vertical", command=self.coeff_tree.yview)
        self.coeff_tree.configure(yscrollcommand=sb2.set)
        self.coeff_tree.pack(side="left", fill="both", expand=True)
        sb2.pack(side="right", fill="y")
        ttk.Label(right, text="M1 地址 0x80~0x9F · NOR 地址 0x00~0x1F",
                  style="Muted.TLabel").grid(row=2, column=0, sticky="w",
                                             pady=(3, 0))

    def _build_item_tab(self, parent) -> None:
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.pack(fill="both", expand=True)

        ttk.Label(card, text="单项命令", style="Head.TLabel").pack(anchor="w")

        row = ttk.Frame(card, style="Card.TFrame")
        row.pack(fill="x", pady=(8, 6))
        ttk.Label(row, text="测量参数", style="Card.TLabel").pack(side="left")
        self.meas_var = tk.StringVar(value="0x0000")
        ttk.Entry(row, textvariable=self.meas_var, width=8).pack(side="left",
                                                                padx=(4, 10))
        ttk.Label(row, text="稳定时间", style="Card.TLabel").pack(side="left")
        self.settle_var = tk.StringVar(value="300")
        ttk.Entry(row, textvariable=self.settle_var, width=5).pack(side="left",
                                                                   padx=(4, 2))
        ttk.Label(row, text="ms", style="Muted.TLabel").pack(side="left",
                                                             padx=(0, 10))
        ttk.Label(row, text="系数路数", style="Card.TLabel").pack(side="left")
        self.count_var = tk.StringVar(value="32")
        ttk.Entry(row, textvariable=self.count_var, width=4).pack(side="left",
                                                                  padx=(4, 0))

        btns = ttk.Frame(card, style="Card.TFrame")
        btns.pack(fill="x", pady=(0, 8))
        for text, action in (("OTP 校验", "otp"),
                             ("输出 3.3 V", "out33"), ("输出 1.8 V", "out18"),
                             ("读系数（按路数）", "coeff"),
                             ("读 M1（按路数）", "m1")):
            ttk.Button(btns, text=text,
                       command=lambda a=action: self._on_cmd(a)).pack(
                side="left", padx=(0, 6))

        fixed = ttk.Frame(card, style="Card.TFrame")
        fixed.pack(fill="x", pady=(0, 8))
        ttk.Label(fixed, text="全 32 路", style="Card.TLabel").pack(side="left")
        ttk.Button(fixed, text="3.3 V 读 NOR（0x209）",
                   command=lambda: self._on_cmd("nor32")).pack(
            side="left", padx=(8, 6))
        ttk.Button(fixed, text="1.8 V 读 M1（0x207）",
                   command=lambda: self._on_cmd("m1_32")).pack(side="left")
        ttk.Label(fixed, text="不用先设压，固件自己切 3.3 V / 1.8 V",
                  style="Muted.TLabel").pack(side="left", padx=(10, 0))

        rail = ttk.Frame(card, style="Card.TFrame")
        rail.pack(fill="x", pady=(0, 6))
        ttk.Label(rail, text="轨压", style="Card.TLabel").pack(side="left")
        self.voltage_var = tk.StringVar(value="3300")
        ttk.Entry(rail, textvariable=self.voltage_var, width=6).pack(
            side="left", padx=(4, 2))
        ttk.Label(rail, text="mV", style="Muted.TLabel").pack(side="left",
                                                              padx=(0, 8))
        ttk.Button(rail, text="设定", width=6,
                   command=self._on_set_supply).pack(side="left")
        self.rail_btn = ttk.Button(rail, text="读取轨压", command=self._on_read_rail)
        self.rail_btn.pack(side="left", padx=(8, 0))
        self.rail_set_btn = ttk.Button(rail, text="设压并回读",
                                       command=self._on_set_and_read_rail)
        self.rail_set_btn.pack(side="left", padx=(8, 0))
        ttk.Checkbutton(rail, text="连续监测（0.5 s）", variable=self._auto_rail_var(),
                        style="Card.TCheckbutton").pack(side="left", padx=(10, 0))

        self.rail_value = tk.Label(rail, text="----", bg=CARD, fg=MUTED,
                                   font=("Consolas", 17, "bold"))
        self.rail_value.pack(side="left", padx=(14, 0))
        self.rail_verdict = ttk.Label(rail, text="尚未读取", style="Muted.TLabel")
        self.rail_verdict.pack(side="left", padx=(8, 0))

        ttk.Label(card,
                  text="提示：输出测试正常结束后会保持供电，可直接点「读取轨压」；"
                       "整套测试 / 读系数 / 读 M1 结束时仍会断电，这时读到接近 0 mV "
                       "是正常的，要看带电轨压请用「设压并回读」。"
                       "M1 bank 走 1.8 V，正常 bank 走 3.3 V；"
                       "「3.3 V 读 NOR」「1.8 V 读 M1」由固件自己切压，不用先设压。",
                  style="Muted.TLabel", wraplength=960).pack(anchor="w", pady=(0, 6))

        self.item_tree = self._tree(card, 8, (
            ("item", "项目", 180, "w"),
            ("value", "数值", 420, "w"),
            ("verdict", "判定", 260, "w"),
        ))
        self.item_tree.pack(fill="both", expand=True)

    def _build_log(self, parent) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill="x")
        head = ttk.Frame(frame)
        head.pack(fill="x")
        ttk.Label(head, text="运行日志").pack(side="left")
        ttk.Button(head, text="清空", width=6,
                   command=self._clear_log).pack(side="right")

        wrap = ttk.Frame(frame)
        wrap.pack(fill="x", pady=(4, 0))
        self.log = tk.Text(wrap, height=5, bg="#ffffff", fg=TEXT, relief="flat",
                           font=("Consolas", 9), wrap="word",
                           padx=10, pady=8, state="disabled")
        scroll = ttk.Scrollbar(wrap, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.log.tag_configure("err", foreground=NG_COLOR)
        self.log.tag_configure("ok", foreground=OK_COLOR)
        self.log.tag_configure("muted", foreground=MUTED)

    # ---------- 日志 ----------

    def _log(self, message: str, tag: str = "") -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{stamp}] {message}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    # ---------- 串口 ----------

    def _refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports
            infos = list(list_ports.comports())
        except Exception:
            infos = []

        def score(port) -> int:
            """接板子的 USB 转串口优先，调试器/蓝牙靠后。"""
            text = f"{port.device} {port.description}".lower()
            if "jlink" in text or "segger" in text:
                return -10
            if "ch340" in text or "usb-serial" in text or "cp210" in text \
                    or "ft232" in text or "usb serial" in text:
                return 10
            if "蓝牙" in text or "bluetooth" in text:
                return -5
            return 0

        infos.sort(key=score, reverse=True)
        ports = [p.device for p in infos]
        self.port_box["values"] = ports
        if ports and (not self.port_var.get() or self.port_var.get() not in ports):
            self.port_var.set(ports[0])
            desc = next((p.description for p in infos if p.device == ports[0]), "")
            self._log(f"默认选中 {ports[0]}（{desc}），不对的话手动改", "muted")
        if not ports:
            self.port_var.set("")
            self._log("没有检测到串口，插好 USB 转串口后点「刷新」", "muted")

    def _toggle_connect(self) -> None:
        if self.master is not None:
            self._disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self._log("请先选择串口", "err")
            return
        try:
            baud = int(self.baud_var.get())
            slave = int(self.slave_var.get())
            self.master = RtuMaster(port, baudrate=baud, slave=slave).open()
        except Exception as exc:  # noqa: BLE001
            self._log(f"打开串口失败：{exc}", "err")
            return

        self.tester = Wf280Tester(self.master)
        self.connect_btn.configure(text="断开")
        self.version_btn.configure(state="normal")
        self.conn_label.configure(text=f"已连接 {port} @ {baud}",
                                  foreground=OK_COLOR)
        self.device_info = f"{port} · {baud} 8N1 · 从机 {slave}"
        self.info_label.configure(text=f"{self.device_info} · 正在读取版本…",
                                  foreground=MUTED)
        self._log(f"已连接 {port}，{baud} 8N1，从机 {slave}", "ok")

        def work():
            bases = self.tester.detect_bases()
            return bases, self.tester.read_version(), self.tester.read_rail()

        def done(result, error):
            if error:
                self._log(f"通讯自检失败：{error}", "err")
                self.info_label.configure(
                    text=f"{self.device_info} · 通讯失败", foreground=NG_COLOR)
                return
            bases, version, rail = result
            self.version_text = version.text if version.ok else "未知"
            self._update_info()
            self._log(f"寄存器基址 命令@{bases[0]} / 结果@{bases[1]}", "muted")
            self._show_rail(rail)

        self._run_async(work, done)

    def _update_info(self) -> None:
        self.info_label.configure(
            text=f"设备：固件 {self.version_text} · {self.device_info}",
            foreground=TEXT)

    def _disconnect(self) -> None:
        if self.master is not None:
            self.master.close()
        self.master = None
        self.tester = None
        self.version_text = "未知"
        self.device_info = ""
        self.connect_btn.configure(text="连接")
        self.version_btn.configure(state="disabled")
        self.conn_label.configure(text="未连接", foreground=MUTED)
        self.info_label.configure(text="未连接设备", foreground=MUTED)
        self._log("已断开串口")

    # ---------- 异步执行 ----------

    def _busy_guard(self) -> bool:
        """有命令在跑时只提示一句，别把上一条结果清掉。"""
        if self.busy:
            self._log("上一条命令还没执行完，请稍等", "muted")
            return True
        return False

    def _run_async(self, work, done) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self.busy:
            return
        self.busy = True

        def runner():
            try:
                self._queue.put(("done", done, (work(), None)))
            except Exception as exc:  # noqa: BLE001
                self._queue.put(("done", done, (None, exc)))

        threading.Thread(target=runner, daemon=True).start()

    def _post_status(self, text: str) -> None:
        self._queue.put(("status", text, None))

    def _drain_queue(self) -> None:
        try:
            while True:
                kind, payload, args = self._queue.get_nowait()
                if kind == "status":
                    self.full_status.configure(text=payload, foreground=ACCENT)
                    continue
                self.busy = False
                try:
                    payload(*args)
                except Exception as exc:  # noqa: BLE001
                    self._log(f"界面回调异常：{exc}", "err")
        except queue.Empty:
            pass
        self.root.after(50, self._drain_queue)

    # ---------- 表格 ----------

    def _show_rows(self, tree: ttk.Treeview, rows, skip=None) -> None:
        if skip:
            rows = [r for r in rows if r[0] not in skip]
        for item, value, verdict in rows:
            if verdict == "OK":
                tag = "ok"
            elif verdict.startswith("NG") or "未" in verdict or "偏离" in verdict:
                tag = "ng"
            else:
                tag = ""
            tree.insert("", "end", values=(item, value, verdict), tags=(tag,))

    def _clear_tree(self, tree: ttk.Treeview) -> None:
        for item in tree.get_children():
            tree.delete(item)

    # ---------- 取输入 ----------

    def _read_params(self):
        try:
            measurement = int(self.meas_var.get().strip(), 0)
            settle = int(self.settle_var.get().strip())
            count = int(self.count_var.get().strip())
        except ValueError:
            self._log("参数填写有误：测量参数可写 0x0000 或 0，其余填十进制", "err")
            return None
        if not 0 <= measurement <= 0xFFFF:
            self._log("测量参数范围 0x0000 ~ 0xFFFF", "err")
            return None
        if not 0 <= settle <= 5000:
            self._log("稳定时间范围 0 ~ 5000 ms", "err")
            return None
        if not 1 <= count <= COEFF_MAX:
            self._log(f"系数路数范围 1 ~ {COEFF_MAX}", "err")
            return None
        return measurement, settle, count

    def _voltage(self):
        try:
            mv = int(self.voltage_var.get().strip())
        except ValueError:
            self._log("轨压请填十进制毫伏值，例如 3300", "err")
            return None
        if not 500 <= mv <= 5000:
            self._log("轨压范围 500 ~ 5000 mV", "err")
            return None
        return mv

    # ---------- ① 整套测试（AllTestFor280）----------

    def _on_full_test(self) -> None:
        self._start_full(
            "轮询 [1,0x400]",
            lambda: self.tester.run_full_test(
                on_tick=lambda e: self._post_status(f"测试中… {e:.1f} s")))

    def _on_full_test_delay(self) -> None:
        self._start_full("固定延时 3 s 后写 [1,0x401]",
                         self.tester.run_full_test_fixed)

    def _start_full(self, note, worker) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        self._set_full_busy(True)
        self.full_status.configure(text="测试中…", foreground=ACCENT)
        self.full_verdict.configure(text="")
        self.coeff_label.configure(text="测试中…")
        self._clear_tree(self.full_tree)
        self._clear_tree(self.coeff_tree)
        self._log(f"触发整套测试 cmd[0]=0x14（AllTestFor280）；{note}")

        started = time.monotonic()

        def work():
            return worker()

        def done(result, error):
            elapsed = time.monotonic() - started
            self._set_full_busy(False)
            if error:
                self.full_status.configure(text="中断", foreground=NG_COLOR)
                self.full_verdict.configure(text="中断", foreground=NG_COLOR)
                self._log(f"整套测试失败：{error}", "err")
                return
            self.full_status.configure(text=f"完成 · {elapsed:.1f} s",
                                       foreground=MUTED)
            self._show_full_result(result)

        self._run_async(work, done)

    def _set_full_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        for btn in (self.full_btn, self.full_delay_btn):
            btn.configure(state=state)

    def _show_full_result(self, result: FullTestResult) -> None:
        self._clear_tree(self.full_tree)
        self._show_rows(self.full_tree, result.rows())
        self._show_coeffs(result)

        if result.ok:
            text = "PASS"
        else:
            text = result.verdict_text()
        self.full_verdict.configure(
            text=text, foreground=OK_COLOR if result.ok else NG_COLOR)

        if result.ok:
            self._log(
                f"AllTestFor280 完成：PASS，轨压 1.8 V {result.rail_1v8_mv} mV / "
                f"3.3 V {result.rail_mv} mV，"
                f"工作 {result.work_ua:.2f} uA（0x{result.work_raw:08X}），"
                f"休眠 {result.sleep_ua:.2f} uA（0x{result.sleep_raw:08X}），"
                f"M1/NOR 32 路一致", "ok")
        else:
            self._log(f"AllTestFor280 完成：{result.verdict_text()}", "err")

    def _show_coeffs(self, result: FullTestResult) -> None:
        self._clear_tree(self.coeff_tree)
        pairs = result.coeff_pairs()
        for index, m1, nor, same in pairs:
            if m1 is None:
                values = (f"{index + 1:02d}", "—", "—", "—")
                tag = ""
            else:
                values = (f"{index + 1:02d}", f"0x{m1:04X}", f"0x{nor:04X}",
                          "一致" if same else "不一致")
                tag = "ok" if same else "ng"
            self.coeff_tree.insert("", "end", values=values, tags=(tag,))

        if not result.coeff_available:
            self.coeff_label.configure(text="失败帧不带系数",
                                       foreground=NG_COLOR)
        elif result.coeff_mismatch is None:
            self.coeff_label.configure(text="32/32 一致", foreground=OK_COLOR)
        else:
            self.coeff_label.configure(
                text=f"第 {result.coeff_mismatch + 1} 路不一致",
                foreground=NG_COLOR)

    # ---------- ② 轨压 ----------

    def _on_read_rail(self) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        self._run_async(self.tester.read_rail, self._show_rail)

    def _on_set_and_read_rail(self) -> None:
        """先按输入框设压，等轨稳定再回读 —— 想看带电轨压就用这个。"""
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        mv = self._voltage()
        if mv is None:
            return
        self.rail_value.configure(text="读取中…", fg=MUTED)
        self._log(f"设压 {mv} mV 后回读轨压")

        def work():
            self.tester.set_supply(mv)
            time.sleep(0.35)
            return self.tester.read_rail()

        self._run_async(work, self._show_rail)

    def _show_rail(self, result: RailReading | None, error=None) -> None:
        if error:
            self.rail_value.configure(text="----", fg=MUTED)
            self.rail_verdict.configure(text=f"读取失败：{error}")
            self._log(f"轨压读取失败：{error}", "err")
            return
        self.rail_value.configure(text=f"{result.mv} mV",
                                  fg=OK_COLOR if result.ok else NG_COLOR)
        self.rail_verdict.configure(text=result.verdict)

    def _auto_rail_var(self) -> tk.BooleanVar:
        if not hasattr(self, "auto_rail"):
            self.auto_rail = tk.BooleanVar(value=False)
        return self.auto_rail

    def _poll_auto_rail(self) -> None:
        if self.auto_rail.get() and self.tester is not None and not self.busy:
            self._on_read_rail()
        self.root.after(500, self._poll_auto_rail)

    # ---------- ② 单项命令 ----------

    def _on_cmd(self, action: str) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        measurement = settle = count = 0
        if action in ("out33", "out18", "coeff", "m1"):
            params = self._read_params()
            if params is None:
                return
            measurement, settle, count = params

        jobs = {
            "otp": ("OTP 校验", lambda: self.tester.read_otp()),
            "out33": (f"输出 3.3 V（{measurement:#06x} / {settle} ms）",
                      lambda: self.tester.test_output_3v3(measurement, settle)),
            "out18": (f"输出 1.8 V（{measurement:#06x} / {settle} ms）",
                      lambda: self.tester.test_output_1v8(measurement, settle)),
            "coeff": (f"读系数（{count} 路）", lambda: self.tester.read_coeff(count)),
            "m1": (f"读 M1（{count} 路）", lambda: self.tester.read_m1(count)),
            "nor32": ("3.3 V 读 NOR（32 路）",
                      lambda: self.tester.read_coeff(COEFF_MAX)),
            "m1_32": ("1.8 V 读 M1（32 路）",
                      lambda: self.tester.read_m1(COEFF_MAX)),
        }
        label, work = jobs[action]

        self._clear_tree(self.item_tree)
        self._log(f"执行：{label}")
        self._run_async(work, lambda r, e: self._show_cmd(label, r, e))

    def _show_cmd(self, label: str, result, error) -> None:
        if error:
            self._log(f"{label} 失败：{error}", "err")
            return
        self._show_rows(self.item_tree, result.rows())
        if getattr(result, "ok", True):
            self._log(f"{label} 完成：OK", "ok")
        else:
            self._log(f"{label} 完成：NG", "err")

    def _on_set_supply(self) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        mv = self._voltage()
        if mv is None:
            return
        self._log(f"设定 DAC 轨压 {mv} mV")
        self._run_async(lambda: _Simple(self.tester.set_supply(mv)),
                        lambda r, e: self._show_set_supply(mv, r, e))

    def _show_set_supply(self, mv: int, result, error) -> None:
        if error:
            self._log(f"设定电压失败：{error}", "err")
            return
        if result.ok:
            self._log(f"轨压已设为 {mv} mV", "ok")
        else:
            self._log(f"设定电压失败：{status_text(result.status)}", "err")

    def _on_read_version(self) -> None:
        if self.tester is None:
            self._log("请先连接串口", "err")
            return
        if self._busy_guard():
            return
        self._run_async(self.tester.read_version, self._show_version)

    def _show_version(self, version, error) -> None:
        if error:
            self._log(f"版本读取失败：{error}", "err")
            return
        self.version_text = version.text if version.ok else "未知"
        self._update_info()
        self._log(f"设备固件版本 {version.text}", "ok" if version.ok else "err")


def run() -> int:
    root = tk.Tk()
    TestApp(root)
    root.mainloop()
    return 0
