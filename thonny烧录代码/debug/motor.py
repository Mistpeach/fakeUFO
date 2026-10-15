# motor.py - 2in1 调试版
# =============================================================================
# 与 2in1/motor.py 的区别：驱动逻辑、引脚、PWM 频率完全一样，
# 只是额外记录了"当前运行状态"，供串口实时输出使用。
#
# 关键约束：rorate() / stop() 会被 20ms 的定时器回调调用，
#           所以它们只做整数赋值和引脚操作，不 print、不分配字符串。
#           需要文字描述时，由主循环调用 status_str() / pin_str()。
# =============================================================================

from machine import Pin, PWM
from micropython import const

_DIR_FWD = const(0)
_DIR_REV = const(1)
_DIR_STOP = const(2)
_DIR_NAME = ('FWD', 'REV', 'STOP')


class motor:
    def __init__(self, pin1, pin2, pwm_pin, stby_pin=None, name="M?", max_duty=1023):
        """
        :param pin1/pin2: 方向控制引脚 (TB6612 的 IN1/IN2)
        :param pwm_pin:   速度控制引脚 (PWM)
        :param stby_pin:  驱动器 STBY 引脚（传入后会在初始化时拉高使能）
        :param name:      串口输出里显示的名字，如 "M1"
        :param max_duty:  PWM 满量程（ESP32 默认 10bit = 1023）
        """
        self.name = name
        self.max_duty = max_duty

        self.p1 = Pin(pin1, Pin.OUT)
        self.p2 = Pin(pin2, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin, Pin.OUT))
        self.pwm.freq(20000)           # 20kHz，超出人耳范围，消除电机啸叫

        self.stby_pin = stby_pin
        if self.stby_pin:
            self.stby_pin.on()

        # --------------------- 运行状态（全是整数） ---------------------
        self.dir = _DIR_STOP       # _DIR_FWD / _DIR_REV / _DIR_STOP
        self.speed = 0             # 0~127（协议里的速度档位）
        self.duty = 0              # 0~max_duty（真正写进 PWM 的值）
        self.cmd = 0               # 最后一次收到的原始控制字节
        self.applies = 0           # rorate()/stop() 被调用次数（含重复应用）
        self.changes = 0           # 状态真正发生变化的次数（用来判断"该打印了"）

        self.stop()

    # ------------------------------ 内部 ------------------------------
    def _set_state(self, d, spd, duty, cmd):
        """只改整数，不做任何分配 —— 可以安全地在定时器回调里调用。"""
        if d != self.dir or spd != self.speed or duty != self.duty:
            self.changes += 1
        self.dir = d
        self.speed = spd
        self.duty = duty
        self.cmd = cmd

    # ------------------------------ 对外 ------------------------------
    def deinit(self):
        """释放 PWM 资源"""
        self.pwm.deinit()

    def stop(self):
        """停止电机：两个方向引脚都拉低（滑行停止）+ PWM 占空比 0"""
        self.p1.off()
        self.p2.off()
        self.pwm.duty(0)
        self.applies += 1
        self._set_state(_DIR_STOP, 0, 0, 0)

    def rorate(self, value):
        """
        控制电机旋转（行为与 2in1 原版一致）
        :param value: bit7=方向(1=反转)，bit0~6=速度(0~127)
        """
        speed_raw = value & 0x7f
        speed_duty = (speed_raw * self.max_duty) // 127

        if value & 0x80:               # 最高位为 1 -> 反转
            self.p1.off()
            self.p2.on()
            d = _DIR_REV
        else:                          # 最高位为 0 -> 正转
            self.p1.on()
            self.p2.off()
            d = _DIR_FWD

        self.pwm.duty(speed_duty)
        self.applies += 1

        # 速度档位为 0 时对使用者来说就是"停止"，这样串口里更直观；
        # 引脚电平仍按 bit7 设置，原始字节保存在 self.cmd 里，不会丢信息。
        if speed_raw == 0:
            d = _DIR_STOP
        self._set_state(d, speed_raw, speed_duty, value)

    # --------------------- 以下两个方法会分配字符串 ---------------------
    # 只允许在主循环（非中断上下文）里调用。
    def status_str(self):
        """一行紧凑的运行状态，例如： M1 FWD  63/127 duty= 507"""
        return "%s %s %3d/127 duty=%4d" % (
            self.name, _DIR_NAME[self.dir], self.speed, self.duty)

    def pin_str(self):
        """方向引脚的实际电平，排查接线/方向反了的时候很有用"""
        return "IN=%d,%d" % (self.p1.value(), self.p2.value())
