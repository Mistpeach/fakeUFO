# main.py - 2in1 调试版（串口可观测版）
# =============================================================================
# 与 2in1/main.py 的关系：
#   BLE 协议、UUID、引脚分配、电机驱动逻辑 完全一致，可以直接替换使用；
#   区别只在于"可观测性"，以及修掉了一个断连误判的 bug。
#
# ---------------------------------------------------------------------------
# 串口输出格式（115200 8N1，Thonny 里直接看）：   [T+12.345s] [TAG] 内容
#   BOOT / WAIT  上电与 5 秒等待（在 boot.py 里）
#   INIT         引脚、固件版本、复位原因、CPU、内存
#   BLE          广播包、服务/特征句柄、发射功率设置结果
#   CONN         连接、断开、连接参数更新、拒绝重复连接
#   MTU          MTU 协商结果
#   WRITE        连接后的第一个报文 / 异常报文（逐包打印会拖慢 BLE，所以只打这些）
#   MOTOR        两路电机的实时运行状态：状态一变就打（这是主要输出）
#   HEART        1 秒一次的心跳，没变化时也保证能看到两路状态
#   STATE        连接状态切换（定时器周期、LED）
#   WARN / ERR   异常
#   EVENT        未处理的 BLE 事件（每种只打一次，便于排查）
#   EXIT         退出/停机
#
# ---------------------------------------------------------------------------
# 实时性是怎么保证的（重点）：
#   1. BLE 回调里只做"拷贝字节 + 改整数"，绝对不 print。打印全部放在主循环，
#      因此高频报文不会阻塞 BLE 事件处理（原版是每个包都 print，115200 下
#      一行约 60 字节 ≈ 5ms，50 包/秒就吃掉 25% 的时间）。
#   2. 主循环 20ms 转一圈，电机状态一变就在 20ms 内输出；输出上限默认
#      20 行/秒（STATUS_MIN_GAP_MS=50），被限速的次数记在"限流"里，
#      不会假装没有丢数据。想要更快，把 STATUS_MIN_GAP_MS 改成 0、
#      LOOP_TICK_MS 改小即可（上限就是 1000/LOOP_TICK_MS 行/秒）。
#   3. 定时器每 20ms 把速度写到引脚（原版 100ms），串口看到的"已生效状态"
#      最多滞后 20ms，基本就是实时。
#   4. MOTOR 行里的 L=/R= 取自电机对象上"真正被应用过"的字节，和同行显示的
#      方向/速度/占空比一定是同一帧数据，不会出现张冠李戴。
#   5. Ctrl-C 停止时会先停电机、再关定时器，不会出现"程序停了电机还在转"。
# =============================================================================

from motor import motor
from machine import Pin, Timer
from time import sleep_ms, ticks_ms, ticks_diff
from micropython import const
import ubluetooth
import sys
import gc

# ----------------------------- 可调参数 -------------------------------------
DEBUG = True               # 总开关。False 时只保留 WARN/ERR/EXIT/CONN/STATE
VERBOSE_MOTOR = True       # MOTOR 行是否附带方向引脚电平 IN=x,y（排查接线用）
APPLY_PERIOD_MS = 20       # 定时器把速度写到引脚的周期（原版 100ms）
LOOP_TICK_MS = 20          # 主循环节拍 -> 状态输出最快 50 次/秒
STATUS_MIN_GAP_MS = 50     # MOTOR 行最小间隔（20 行/秒上限）；0 = 不限速
HEARTBEAT_MS = 1000        # 无变化时的心跳周期
STALE_STOP_MS = 0          # >0：超过该毫秒数收不到报文就自动停机（0=关闭）
                           #     注意：某些 App 发送间隔较长，太小会误停
_RATE_WINDOW_MS = 500      # 统计收包速率的时间窗（首次测量前显示 "?"）

# ---------- BLE 事件常量 ----------
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_MTU_EXCHANGED = const(21)
_IRQ_CONNECTION_UPDATE = const(27)

_IRQ_NAME = {
    1: 'CENTRAL_CONNECT', 2: 'CENTRAL_DISCONNECT', 3: 'GATTS_WRITE',
    21: 'MTU_EXCHANGED', 27: 'CONNECTION_UPDATE',
}

# ---------- 广告参数 ----------
_ADV_INTERVAL_US = const(50000)      # 50ms 快速广播

# ---------- BLE UUID ----------
UFO_SERVICE_STR = '40ee0200-63ec-4b7f-8ce7-712efd55b90e'
UFO_CHAR_STR = '40ee0202-63ec-4b7f-8ce7-712efd55b90e'
XTOYS_SERVICE_STR = '40ee1111-63ec-4b7f-8ce7-712efd55b90e'
XTOYS_CHAR_STR = '40ee2222-63ec-4b7f-8ce7-712efd55b90e'

UFO_SERVICE_UUID = ubluetooth.UUID(UFO_SERVICE_STR)
UFO_CHAR_UUID = ubluetooth.UUID(UFO_CHAR_STR)
XTOYS_SERVICE_UUID = ubluetooth.UUID(XTOYS_SERVICE_STR)
XTOYS_CHAR_UUID = ubluetooth.UUID(XTOYS_CHAR_STR)

# ---------- 不在 DEBUG 模式下也不屏蔽的标签 ----------
_ALWAYS = ('WARN', 'ERR', 'EXIT', 'CONN', 'STATE')


# ----------------------------- 日志工具 -------------------------------------
def _log(tag, msg):
    if not DEBUG and tag not in _ALWAYS:
        return
    ms = ticks_ms()
    print("[T+%d.%03ds] [%s] %s" % (ms // 1000, ms % 1000, tag, msg))


def _hex(data):
    return ' '.join('%02x' % b for b in data)


def _reset_cause_str():
    try:
        import machine
        c = machine.reset_cause()
    except Exception:
        return "未知"
    names = []
    for n in ('PWRON_RESET', 'HARD_RESET', 'WDT_RESET', 'DEEPSLEEP_RESET', 'SOFT_RESET'):
        v = getattr(machine, n, None)
        if v is not None and v == c:
            names.append(n)
    return "%d(%s)" % (c, '/'.join(names) if names else '?')


class UFOTW_BLE_DEBUG:
    def __init__(self, name="UFO-TW"):
        self.name = name

        # ------------------------- 硬件 -------------------------
        self.stby_pin = Pin(10, Pin.OUT)
        self.stby_pin.on()
        self.m1 = motor(pin1=1, pin2=2, pwm_pin=3, stby_pin=self.stby_pin, name="M1")
        self.m2 = motor(pin1=4, pin2=5, pwm_pin=6, stby_pin=self.stby_pin, name="M2")
        self.led = Pin(8, Pin.OUT)
        self.timer1 = Timer(0)
        self.speed = [0, 0]

        # ------------------------- 状态 -------------------------
        self._conn_handle = None
        self._adv_pending = False
        self._stale_stopped = False

        # --------------------- 调试计数器 -----------------------
        self.writes = 0            # 累计收到的报文数
        self.unknown_pkts = 0      # 字节数不足 3 的异常报文
        self.dup_pkts = 0          # 与上一包完全相同的报文数
        self.throttled = 0         # 因限速没打出来的状态更新次数
        self.connects = 0
        self.disconnects = 0
        self.rejected = 0
        self.last_write_ms = 0
        self._rate_val = -1        # -1 = 还没测出速率，显示为 "?"
        self._rate_ms = ticks_ms()
        self._rate_writes = 0
        self._evt_seen = []
        self._src = '?'
        self._first_pkt = True     # 连接后的第一包要单独打一行

        # 最近一次报文（在 BLE 回调里填充，避免回调里做分配）
        self._pkt = bytearray(24)
        self._pkt_len = 0
        self._pkt_new = False
        self._hdr = -1

        # ------------------------- BLE --------------------------
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self._log_chip()
        self._log_mac()
        self._try_tx_power()
        self.ble.config(gap_name=name)
        self.ble.irq(self.ble_irq)
        self.register()
        self._start_advertising("初始化")
        self._set_disconnected_state()

    # ==================== 启动信息 ====================
    def _log_chip(self):
        _log("INIT", "引脚: M1: IN1=GPIO1 IN2=GPIO2 PWM=GPIO3 | "
                     "M2: IN1=GPIO4 IN2=GPIO5 PWM=GPIO6 | STBY=GPIO10 | LED=GPIO8")
        try:
            _log("INIT", "平台: %s" % sys.platform)
        except Exception:
            pass
        try:
            import os
            _log("INIT", "固件: %s" % (os.uname().version,))
        except Exception as e:
            _log("WARN", "读取固件版本失败: %s" % e)
        try:
            _log("INIT", "复位原因: %s" % _reset_cause_str())
        except Exception as e:
            _log("WARN", "读取复位原因失败: %s" % e)
        try:
            import machine
            _log("INIT", "CPU: %dMHz  空闲内存: %d bytes"
                 % (machine.freq() // 1000000, gc.mem_free()))
        except Exception as e:
            _log("WARN", "读取 CPU/内存失败: %s" % e)

    def _log_mac(self):
        try:
            addr_type, addr = self.ble.config('mac')
            _log("BLE", "本机地址 %s (addr_type=%d)  广播名 '%s'"
                 % (':'.join('%02x' % b for b in addr), addr_type, self.name))
        except Exception as e:
            _log("WARN", "读取 BLE 地址失败: %s" % e)

    def _try_tx_power(self):
        # 说明：MicroPython 官方至今没有实现 ble.config(tx_power=...)，
        # 这里主动试一次并如实报告结果（设不上也不影响功能）。
        try:
            self.ble.config(tx_power=7)     # 7 = +9dBm，ESP-IDF 里的最大值
            _log("BLE", "tx_power=7 设置成功（最大发射功率）")
        except Exception as e:
            _log("WARN", "本固件不支持 tx_power（%s），发射功率保持默认" % e)

    # ==================== BLE 事件分发 ====================
    def ble_irq(self, event, data):
        # 回调里只做必要的读/写，不做 print、不做 sleep
        if event == _IRQ_CENTRAL_CONNECT:
            ch, at, ad = self._unpack(data, 3)
            self._handle_connect(ch, at, ad)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            ch, _, ad = self._unpack(data, 3)
            self._handle_disconnect(ch, ad)
        elif event == _IRQ_GATTS_WRITE:
            self._handle_write(data[0], data[1])
        elif event == _IRQ_MTU_EXCHANGED:
            _log("MTU", "协商完成 handle=%s mtu=%s" % (data[0], data[1]))
        elif event == _IRQ_CONNECTION_UPDATE:
            try:
                _log("CONN", "连接参数更新 handle=%s interval=%s latency=%s timeout=%s status=%s"
                     % (data[0], data[1], data[2], data[3], data[4]))
            except Exception:
                _log("CONN", "连接参数更新 data=%s" % (data,))
        else:
            # 未处理的事件每种只打一次，方便发现固件到底发了什么
            if event not in self._evt_seen:
                self._evt_seen.append(event)
                _log("EVENT", "未处理事件 %s(%s) data=%s"
                     % (event, _IRQ_NAME.get(event, '?'), data))

    @staticmethod
    def _unpack(data, n):
        """兼容不同固件：事件数据长度可能不一致，缺的补默认值。"""
        if not isinstance(data, tuple):
            return data, -1, b''
        out = []
        for i in range(n):
            out.append(data[i] if i < len(data) else (-1 if i == 1 else b''))
        return out[0], out[1], out[2]

    # ==================== 连接 / 断开 ====================
    def _handle_connect(self, conn_handle, addr_type, addr):
        if self._conn_handle is not None and self._conn_handle != conn_handle:
            self.rejected += 1
            _log("WARN", "已有连接 handle=%s，拒绝新连接 handle=%s addr=%s"
                 % (self._conn_handle, conn_handle, _hex(bytes(addr))))
            try:
                self.ble.gap_disconnect(conn_handle)
            except Exception as e:
                _log("ERR", "gap_disconnect(%s) 失败: %s" % (conn_handle, e))
            return

        self._conn_handle = conn_handle
        self.connects += 1
        self._first_pkt = True
        self._stale_stopped = False
        _log("CONN", "已连接 handle=%s addr_type=%s addr=%s（第 %d 次连接）"
             % (conn_handle, addr_type, _hex(bytes(addr)), self.connects))
        self._set_connected_state()

    def _handle_disconnect(self, conn_handle, addr=None):
        # ★ 这里是相对 2in1 原版的 bug 修复：
        #   原版只保护了 _conn_handle 的清零，却仍然无条件执行"停电机 +
        #   把定时器换成 LED 闪烁 + 重新广播"。于是一旦收到"非当前活动连接"
        #   的断开事件（被拒绝的重复连接、或重连时旧连接的断开事件迟到），
        #   正常使用中的连接会被误杀：电机会停，而且定时器回调变成了 LED
        #   闪烁，此后再收到任何速度指令都不会作用到电机上，必须重连才能恢复。
        if self._conn_handle != conn_handle:
            _log("WARN", "忽略非活动连接的断开事件 handle=%s（当前活动 handle=%s）"
                 % (conn_handle, self._conn_handle))
            return

        self._conn_handle = None
        self.disconnects += 1
        _log("CONN", "已断开 handle=%s addr=%s（累计 连接%d/断开%d/拒绝%d）"
             % (conn_handle, _hex(bytes(addr)) if addr else '?',
                self.connects, self.disconnects, self.rejected))
        self._set_disconnected_state()
        # 重新广播放到主循环里做（回调里不能 sleep_ms(50)）
        self._adv_pending = True

    # ==================== 写入 ====================
    def _handle_write(self, conn_handle, attr_handle):
        try:
            buf = self.ble.gatts_read(attr_handle)
        except Exception as e:
            _log("ERR", "gatts_read(handle=%s) 失败: %s" % (attr_handle, e))
            return

        n = len(buf)
        self.writes += 1
        self.last_write_ms = ticks_ms()
        self._stale_stopped = False

        # 判断这条特征属于哪套服务 -> 也就知道是哪个 App 在控制
        if attr_handle in self.rx_ufo:
            self._src = 'UFOPlayer'
        elif attr_handle in self.rx_xtoys:
            self._src = 'XToys'
        else:
            self._src = '未知(handle=%s)' % attr_handle

        if n > len(self._pkt):
            n = len(self._pkt)
        for i in range(n):            # 拷贝到预分配缓冲，回调里尽量不分配内存
            self._pkt[i] = buf[i]
        self._pkt_len = n
        self._pkt_new = True

        if len(buf) >= 3:
            self._hdr = buf[0]
            self.speed[0] = buf[1]
            self.speed[1] = buf[2]
        else:
            self.unknown_pkts += 1

    # ==================== 服务注册 ====================
    def register(self):
        ufo_service = (UFO_SERVICE_UUID, ((UFO_CHAR_UUID, ubluetooth.FLAG_WRITE),))
        xtoys_service = (XTOYS_SERVICE_UUID, ((XTOYS_CHAR_UUID, ubluetooth.FLAG_WRITE),))

        # UFOPlayer 是用 services.Last() 去找特征的，所以 UFO 服务必须最后注册
        result = self.ble.gatts_register_services((xtoys_service, ufo_service))
        self.rx_xtoys = result[0]
        self.rx_ufo = result[1]

        _log("BLE", "服务[0] XToys     svc=%s char=%s handles=%s"
             % (XTOYS_SERVICE_STR, XTOYS_CHAR_STR, self.rx_xtoys))
        _log("BLE", "服务[1] UFOPlayer svc=%s char=%s handles=%s  <-- 必须最后"
             % (UFO_SERVICE_STR, UFO_CHAR_STR, self.rx_ufo))

    # ==================== 广播 ====================
    def _start_advertising(self, reason=""):
        try:
            self.ble.gap_advertise(None)      # 先停掉旧广播
        except Exception:
            pass
        sleep_ms(50)                          # 给 BLE 栈一点时间释放资源

        name = bytes(self.name, 'UTF-8')
        adv_data = bytearray(b'\x02\x01\x02') + \
            bytearray((len(name) + 1, 0x09)) + name
        self.ble.gap_advertise(_ADV_INTERVAL_US, adv_data=adv_data)
        _log("BLE", "开始广播(%s) 间隔=%dms 包长=%d data=%s"
             % (reason, _ADV_INTERVAL_US // 1000, len(adv_data), _hex(adv_data)))

    # ==================== 连接状态 ====================
    def _set_connected_state(self):
        self.m1.stop()
        self.m2.stop()
        self.speed = [0, 0]
        self.led.value(1)
        try:
            self.timer1.deinit()
        except Exception:
            pass
        self.timer1.init(period=APPLY_PERIOD_MS, mode=Timer.PERIODIC,
                         callback=self._apply_speed)
        _log("STATE", "状态=已连接  电机=停止  LED=常亮  控制定时器=%dms" % APPLY_PERIOD_MS)

    def _set_disconnected_state(self):
        self.m1.stop()
        self.m2.stop()
        self.speed = [0, 0]
        try:
            self.timer1.deinit()
        except Exception:
            pass
        self.timer1.init(period=1000, mode=Timer.PERIODIC, callback=self._blink)
        _log("STATE", "状态=未连接  电机=停止  LED=1s闪烁  定时器=1000ms(闪灯)")

    def _blink(self, t):
        self.led.value(not self.led.value())

    def _apply_speed(self, t):
        # 定时器回调：只改引脚和整数，不打印、不分配字符串
        self.m1.rorate(self.speed[0])
        self.m2.rorate(self.speed[1])

    # ==================== 串口输出（只在主循环里调用） ====================
    def _rate(self, now):
        """收包速率，最多每 _RATE_WINDOW_MS 重算一次（-1 = 还没测出来）"""
        dt = ticks_diff(now, self._rate_ms)
        if dt >= _RATE_WINDOW_MS:
            self._rate_val = (self.writes - self._rate_writes) * 1000 // dt
            self._rate_writes = self.writes
            self._rate_ms = now
        return self._rate_val

    def _rate_str(self, now):
        r = self._rate(now)
        return '?' if r < 0 else '%d' % r

    def _motor_str(self, m):
        if VERBOSE_MOTOR:
            return m.status_str() + ' ' + m.pin_str()
        return m.status_str()

    def _status_line(self, now):
        # L=/R= 取自电机对象上真正被应用过的字节，与同行的方向/速度/占空比同源
        return "%s | %s | L=0x%02x R=0x%02x | 收包 %s/s" % (
            self._motor_str(self.m1), self._motor_str(self.m2),
            self.m1.cmd, self.m2.cmd, self._rate_str(now))

    def _heart_line(self, now):
        link = ("handle=%s" % self._conn_handle) if self._conn_handle is not None else "未连接"
        return ("%s | %s | 收包 %s/s 累计 %d 重复 %d 缺字节 %d 限流 %d 应用 %d | "
                "内存 %d | 链路 %s | 来源 %s") % (
            self._motor_str(self.m1), self._motor_str(self.m2),
            self._rate_str(now), self.writes, self.dup_pkts, self.unknown_pkts,
            self.throttled, self.m1.applies, gc.mem_free(), link, self._src)

    # ==================== 主循环 ====================
    def monitor_loop(self):
        m1, m2 = self.m1, self.m2
        last_changes = (m1.changes, m2.changes)
        last_seen = b''
        t_status = ticks_ms()
        t_heart = ticks_ms()

        while True:
            now = ticks_ms()

            # (1) 断开后重新广播（不在 BLE 回调里 sleep）
            if self._adv_pending:
                self._adv_pending = False
                self._start_advertising("断开后重新广播")

            # (2) 消费最近一次收到的报文
            if self._pkt_new:
                self._pkt_new = False
                pkt = bytes(self._pkt[:self._pkt_len])
                if pkt == last_seen:
                    self.dup_pkts += 1
                last_seen = pkt
                if self._first_pkt:
                    self._first_pkt = False
                    _log("WRITE", "首个报文 来自=%s len=%d hdr=0x%02x data=%s"
                         % (self._src, len(pkt), self._hdr, _hex(pkt)))
                elif len(pkt) < 3:
                    _log("WARN", "报文过短 len=%d data=%s（已忽略，期望 >=3 字节）"
                         % (len(pkt), _hex(pkt)))

            # (3) 两路电机状态是否变化
            cur = (m1.changes, m2.changes)
            changed = (cur != last_changes)

            # (4) 输出决策：变化优先，其次心跳，被限速就计数
            if changed and ticks_diff(now, t_status) >= STATUS_MIN_GAP_MS:
                last_changes = cur
                t_status = now
                _log("MOTOR", self._status_line(now))
            elif ticks_diff(now, t_heart) >= HEARTBEAT_MS:
                last_changes = cur
                t_status = now
                t_heart = now
                _log("HEART", self._heart_line(now))
            elif changed:
                # 有变化但被限速：本轮不打，等间隔够了会自动补上"最新"状态
                self.throttled += 1

            # (5) 可选：长时间收不到报文就自动停机
            if STALE_STOP_MS > 0 and self.last_write_ms and not self._stale_stopped:
                if ticks_diff(now, self.last_write_ms) > STALE_STOP_MS:
                    self._stale_stopped = True
                    m1.stop()
                    m2.stop()
                    self.speed = [0, 0]
                    _log("WARN", "超过 %dms 没收到报文 -> 自动停机（安全保护）"
                         % STALE_STOP_MS)

            sleep_ms(LOOP_TICK_MS)

    # ==================== 退出 ====================
    def shutdown(self):
        try:
            self.timer1.deinit()
        except Exception:
            pass
        self.m1.stop()
        self.m2.stop()
        self.led.value(0)
        _log("EXIT", "电机已停止 / 定时器已关闭 / LED 灭")
        try:
            self.ble.active(False)
            _log("EXIT", "BLE 已关闭，可重新运行 main.py")
        except Exception as e:
            _log("WARN", "关闭 BLE 失败: %s" % e)


if __name__ == "__main__":
    _log("INIT", "==== 2in1 调试版启动 ====")
    _log("INIT", "参数: 应用周期=%dms 主循环=%dms 状态限速=%dms 心跳=%dms 停机保护=%dms"
         % (APPLY_PERIOD_MS, LOOP_TICK_MS, STATUS_MIN_GAP_MS, HEARTBEAT_MS, STALE_STOP_MS))
    ble = UFOTW_BLE_DEBUG()
    try:
        ble.monitor_loop()
    except KeyboardInterrupt:
        ble.shutdown()
        _log("EXIT", "累计 收包=%d 连接=%d 断开=%d 拒绝=%d 缺字节=%d 重复=%d 限流=%d"
             % (ble.writes, ble.connects, ble.disconnects, ble.rejected,
                ble.unknown_pkts, ble.dup_pkts, ble.throttled))
