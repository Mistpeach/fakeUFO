# boot.py - 2in1 调试版
# =============================================================================
# 上电流程（MicroPython 先执行 boot.py，执行完才执行 main.py）：
#   1) 立刻把 GPIO 0~10 全部设为输出并拉低
#      -> TB6612 的 IN1/IN2 不会浮空，STBY=LOW 时驱动器整体待机，
#         上电瞬间电机绝对不会乱转；
#   2) 串口打印平台/固件信息；
#   3) 倒计时 BOOT_DELAY_S 秒（LED 每秒亮 80ms，串口每秒一行）；
#   4) 倒计时结束，才交给 main.py 去开 BLE 和电机控制。
#
# 为什么要等 5 秒：
#   * 给电源和驱动器一个稳定时间；
#   * 打开串口监视器（Thonny）来得及，能看到完整的启动日志；
#   * 这 5 秒里 BLE 还没开，手机 App 搜不到设备，属于正常现象。
#
# 关于 Ctrl-C：
#   在倒计时期间按 Ctrl-C，本文件会捕获并"立即结束等待"，启动流程继续，
#   不会被打断。如果想让程序彻底停下来、回到 REPL 提示符，请在 main.py
#   运行期间再按一次 Ctrl-C（main.py 会先停电机再退出）。
# =============================================================================

from machine import Pin
from time import sleep_ms, ticks_ms
import sys

BOOT_DELAY_S = 5                       # <== 上电等待秒数，改成 0 就不等待
PIN_LOW_LIST = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
LED_PIN = 8
STBY_PIN = 10


def _log(tag, msg):
    ms = ticks_ms()
    print("[T+%d.%03ds] [%s] %s" % (ms // 1000, ms % 1000, tag, msg))


# ---------------------------------------------------------------------------
# 1) 安全初始化：所有相关引脚拉低
# ---------------------------------------------------------------------------
_keep = []                             # 持有 Pin 对象引用，避免被 GC 回收
for _p in PIN_LOW_LIST:
    _pin = Pin(_p, Pin.OUT)
    _pin.value(0)
    _keep.append(_pin)

_log("BOOT", "GPIO %s 全部拉低为输出低电平" % (list(PIN_LOW_LIST),))
_log("BOOT", "STBY(GPIO%d)=LOW -> TB6612 待机中，电机被硬件禁止" % STBY_PIN)

# ---------------------------------------------------------------------------
# 2) 启动信息
# ---------------------------------------------------------------------------
_log("BOOT", "平台: %s" % sys.platform)
try:
    import os
    _log("BOOT", "固件: %s" % (os.uname().version,))
except Exception as _e:
    _log("WARN", "读取固件版本失败: %s" % _e)

# ---------------------------------------------------------------------------
# 3) 倒计时
# ---------------------------------------------------------------------------
led = Pin(LED_PIN, Pin.OUT)
led.value(0)

if BOOT_DELAY_S > 0:
    _log("WAIT", "延时 %d 秒后启动 main.py（此期间 BLE 未开启，App 搜不到设备）"
         % BOOT_DELAY_S)
    _log("WAIT", "此期间按 Ctrl-C 可立即跳过等待")
    _skipped = False
    try:
        for _i in range(BOOT_DELAY_S, 0, -1):
            led.value(1)               # 每秒闪一下，没有串口也能看出在等待
            sleep_ms(80)
            led.value(0)
            _log("WAIT", "倒计时 %d 秒   电机=停止(STBY=LOW)" % _i)
            sleep_ms(920)
    except KeyboardInterrupt:
        _skipped = True
        led.value(0)
        _log("WAIT", "收到 Ctrl-C，跳过剩余等待")
    if not _skipped:
        _log("BOOT", "等待结束")
else:
    _log("BOOT", "BOOT_DELAY_S=0，不等待")

# ---------------------------------------------------------------------------
# 4) 交给 main.py
# ---------------------------------------------------------------------------
_log("BOOT", ">>> 进入 main.py")
