from machine import Pin, Timer, PWM, RTC, reset
from umodbus.serial import Serial as ModbusRTUMaster
import network
import utime as time
from config import loadConfig, writeConfig
import random
import uasyncio as asyncio
from PID import PID


config = loadConfig()
position = 0


class WiFi:
    def __init__(self, LEDPin:int = 2) -> None:
        self.LED=Pin(LEDPin, Pin.OUT) #初始化 WIFI 指示灯
        self.WLAN = network.WLAN(network.STA_IF) #STA 模式
        self.SSID = None
        self.password = None
        self.socket = None
    def connect(self, SSID=config["Network_SSID"], password=config["Network_Password"], fromAdHoc=False):
        if fromAdHoc:
            self.WLAN.disconnect()
            self.WLAN.active(False)
            self.WLAN = network.WLAN(network.STA_IF) #STA 模式
        if SSID or password:
            self.SSID = SSID
            self.password = password
        self.WLAN.active(True)
        self.startTime = time.time()
        if not self.WLAN.isconnected():
            print('Connecting to network...')
            self.WLAN.connect(self.SSID, self.password) #输入 WIFI 账号密码
            while not self.WLAN.isconnected():
                #LED 闪烁提示
                self.LED.value(1)
                time.sleep_ms(300)
                self.LED.value(0)
                time.sleep_ms(300)
                #超时判断,15秒没连接成功判定为超时
                if time.time()-self.startTime > 15 :
                    print('WIFI Connection Timeout! Resetting!')
                    reset()
                    break
        if self.WLAN.isconnected(): #LED 点亮
            self.LED.value(1)            #串口打印信息
            print('Network information:', self.WLAN.ifconfig())
            return True
        else:
            return False
    def getInfo(self):
        return self.WLAN.ifconfig()


class Motor:
    """电机，PWM针脚可选"""
    def __init__(self, PWMPin:Pin, motorPinA:Pin, motorPinB:Pin) -> None:
        if PWMPin:
            self.PWMPin = PWMPin
            self.PWMObj = PWM(self.PWMPin)
        self.motorPinA:Pin = motorPinA
        self.motorPinB:Pin = motorPinB
        self.encoder:Encoder = None

class PWMMotorDriver:
    """PWM直流驱动器类，需绑定一个Motor类"""
    """若有绝对值编码器，使用PID效果更好"""
    def __init__(self, Motor1:Motor, Motor2:Motor, debugPin1=None, debugPin2=None) -> None:
        self.Motor1 = Motor1
        self.Motor2 = Motor2
        if debugPin1:
            self.debugPin1 = Pin(debugPin1, Pin.IN, Pin.PULL_DOWN)
            self.debugPin1.irq(handler=lambda pin:self.debugCallback(pin, direction=1))
        if debugPin2:
            self.debugPin2 = Pin(debugPin2, Pin.IN, Pin.PULL_DOWN)
            self.debugPin2.irq(handler=lambda pin:self.debugCallback(pin, direction=-1))
        #rtc.irq(trigger=RTC.ALARM0, handler=lambda t:self.zero(Motor1)) # 暂不对Motor2做自动对零
        pass
    def manualBindMotor(self, Motor:Motor, slot:1|2):
        if slot == 1:
            self.Motor1 = Motor
        if slot == 2:
            self.Motor2 = Motor
    def updateMotor(self, Motor:Motor, direction:-1|0|1|None, duty:int, freq:int=17000):
        if Motor not in [self.Motor1, self.Motor2]:
            print("Motor与本Driver绑定错误")
            return
        directionPinSetting = setMotorPinFromDirection(direction)
        Motor.motorPinA.value(directionPinSetting[0])
        Motor.motorPinB.value(directionPinSetting[1])
        Motor.PWMObj.freq(freq)
        Motor.PWMObj.duty(duty)
        return
    async def updateMotorByDistance(self, Motor:Motor, distance:int, accelerationSpan=1000, duty=512, forceRun=False):
        """Deprecated
        手动移动距离
        当forceRun为1时，将不作PWM限制或使用Multiplier
        """
        # 检查阶段
        if abs(distance) > 0.2 * config['PositionLimit']:
            print("实际的运动距离超过全程的20%, 自动压缩为20%")
        if Motor not in [self.Motor1, self.Motor2]:
            print("Motor与本Driver绑定错误")
            return
        if not Motor.encoder:
            print("Motor无encoder")
            return
        ## 确定方向
        __originalPosition = Motor.encoder.readPosition()
        if distance > 0:
            direction = 1
        else:
            direction = -1
        ## 限定Duty
        if forceRun == False:
            if duty > config["PWMDutyLock"]:
                duty = config["PWMDutyLock"]
        ## 根据高度与最大高度的比例对PWM乘上一个系数
            duty = duty * (__originalPosition / config["PositionLimit"])
        ## 最低Duty
        ## 正反方向Duty比例参数
        if direction == -1:
            duty = duty * 0.4
        ## 最低Duty
        if duty < 200:
            duty = 200
        ## 时间控制
        startTime = time.localtime(time.time())
        __targetTime = time.localtime(time.mktime((
            startTime[0],
            startTime[1],
            startTime[2],
            startTime[3],
            startTime[4],
            startTime[5] + 5,
            startTime[6],
            startTime[7],
            )))
        # 使能阶段
        print(f'Actual Duty {duty}, Actual Distance {distance}' )
        ## 五级加速
        self.updateMotor(Motor, direction, int(duty * 0.2))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.4))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.6))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.8))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty))
        __speedLock = 0
        if config['Core_Name'] == "TapeA_Winch":
            __speedLock = config['TapeASpeedLock']
        if config['Core_Name'] == "TapeB_Drum":
            __speedLock = config['TapeBSpeedLock']
        while True:
            __currentPosition = Motor.encoder.readPosition()
            __currentSpeed = Motor.encoder.readSpeed()
            __now = time.localtime(time.time())
            if __targetTime[5] == __now[5]: # 最多运转8秒
                print("Breaking due to timeout")
                break
            if (__currentPosition - __originalPosition) / direction > (distance / direction - config["AccelerationBuffer"]):
                print("Breaking due to arriving, originalPosition:%s" % __originalPosition)
                break
            if abs(__currentSpeed) > __speedLock:
                print("Break due to SpeedLock")
                break
            if __currentPosition > config["PositionLimit"] or __currentPosition < 0:
                if not forceRun:
                    print("Breaking due to PositionLimit")
                    break
            # if Motor.scale.readForce() > config.ForceLock:
            #   print("Break due to ForceLock")
            #   break 
        ## 五级减速 
        print("Stopping") 
        self.updateMotor(Motor, direction, int(duty * 0.8))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.6))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.4))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, direction, int(duty * 0.2))
        await asyncio.sleep(accelerationSpan * 2 / 10000)
        self.updateMotor(Motor, 0, 0)
        print("Motion Finished")
        return Motor.encoder.readPosition()

    async def updateMotorToPosition(self, Motor:Motor, targetPosition, accelerationSpan=1000, duty=512):
        """以编码器具体的位置来移动，实质上是updateMotorByDistance的封装"""
        # 检查阶段
        if targetPosition not in (range(0, config["PositionLimit"])):
            return Motor.encoder.readPosition()
        __originalPosition = Motor.encoder.readPosition()
        distance = targetPosition - __originalPosition
        await self.updateMotorByDistance(Motor=Motor, distance=distance, accelerationSpan=accelerationSpan, duty=duty)
        while Motor.encoder.readSpeed() != 0:
            continue
        return Motor.encoder.readPosition()
    async def zero(self, Motor:Motor):
        """自动对零， 需搭配自动定时触发"""
        startTime = time.localtime(time.time())
        __targetTime = time.localtime(time.mktime((
            startTime[0],
            startTime[1],
            startTime[2],
            startTime[3],
            startTime[4] + 1,
            startTime[5],
            startTime[6],
            startTime[7],
            )))
        while True:
            __now = time.localtime(time.time())
            if config['Debugging'] == True:
                print("In the process of zeroing, %s, %s" % (__targetTime, __now))
            if __targetTime[4] == __now[4]: # 使执行一分钟下降
                print("Had enough zeroing")
                break
            await self.updateMotorByDistance(Motor, -2000, accelerationSpan=1000, duty=256, forceRun=True)
        # RTC.alarm(RTC.ALARM0, 1000 * 60 * config['AutoZeroInterval'])@@@@
        return Motor.encoder.setZero()
    async def trigger(self, Motor:Motor):
        __currentPosition = Motor.encoder.readPosition()
        try:
            if config['Core_Name'] == "TapeA_Winch":
                if __currentPosition < config['PositionLimit'] * 0.7:
                    print("Forced Up Trigger")
                    await self.updateMotorByDistance(Motor, config['PositionLimit'] * 0.02, accelerationSpan=1000, duty=512, forceRun=0)
                elif __currentPosition > config['PositionLimit'] * 0.95:
                    print("Forced Down Trigger")
                    await self.updateMotorByDistance(Motor, config['PositionLimit'] * -0.02)
                else:
                    print("Random Up Trigger")
                    __direction = random.choice([-1, 1])
                    await self.updateMotorByDistance(Motor, config['PositionLimit'] * 0.02 * __direction)
        except Exception as e:
            print("Failed trigger", e)
    def debugCallback(self, pin:Pin, direction, duty=256, freq=17000):
        """操作按钮使电机运转，仅测试用，仅使用motor1"""
        if config["Debugging"] == False:
            return
        else:
            pin.irq(handler=None)
            print("Debug Button", pin)
            print(direction)
            if pin.value() == 1:
                self.updateMotor(self.Motor1, direction, duty, freq)
            else:
                self.updateMotor(self.Motor1, 0, 0, freq)
            pin.irq(handler=lambda pin:self.debugCallback(pin, direction))

class Encoder:
    """增量编码器，可绑定电机"""
    """容易产生丢步问题"""
    ### https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-22559033090.12.7a2e489aEx3OQr&id=616919012422
    def __init__(self, HallPinA:Pin, HallPinB:Pin, HallPinZ:Pin,speedGaugeTimerPeriod=300, TimerNum=0) -> None:
        ## 基础属性
        self.counter = 0  # 动态的距离测量值，每一秒重置，读数没有实际意义
        self.counterRecord = 0 # 周期更新的距离测量值，每一秒更新一次，读数体现最新的速度
        self.counterPerRound = 0 # 动态的距离测量值，每一圈重置，读数没有意义
        self.counterPerRoundRecord = 0 # 周期更新的巨量测量值，每一圈更新一次，体现当前本圈数得的脉冲数
        self.counterPosition = 0 # 动态的距离测量值，不作重置，体现距离
        self.HallPinA = HallPinA
        self.HallPinB = HallPinB
        self.HallPinZ = HallPinZ
        ## 内置函数
        def IRQ_HallPinA(self, pin): # 霍尔编码器A相触发IRQ
            valueA = self.HallPinA.value()
            valueB = self.HallPinB.value()
            valueZ = self.HallPinZ.value()
            if config["Debug_Encoder"]:
                print("Encoder TriggerA", valueA, valueB, valueZ)
            if valueA == valueB:
                self.counter += 1
                self.counterPosition += 1
                self.counterPerRound += 1
            else:
                self.counter -= 1
                self.counterPosition -= 1
                self.counterPerRound -= 1
        def IRQ_HallPinZ(self, pin):
            self.counterPerRoundRecord = self.counterPerRound
            self.counterPerRound = 0
            valueA = self.HallPinA.value()
            valueB = self.HallPinB.value()
            valueZ = self.HallPinZ.value()
            if config["Debug_Encoder"]:
                print("Encoder TriggerZ", valueA, valueB, valueZ)
            if self.counterRecord > 30:
                self.counterPosition += (360 - self.counterPerRoundRecord)
            if self.counterRecord < -30:
                self.counterPosition -= (360 + self.counterPerRoundRecord)
        ## 内置绑定
        self.HallPinZ.irq(trigger=Pin.IRQ_RISING, handler=lambda pin:IRQ_HallPinZ(self, pin))
        self.HallPinA.irq(trigger=Pin.IRQ_RISING, handler=lambda pin:IRQ_HallPinA(self, pin)) # 以A相上升沿来出发读数更新
        self.speedGaugeTimer = Timer(TimerNum)
        self.speedGaugeTimer.init(period=speedGaugeTimerPeriod, mode=Timer.PERIODIC, callback=lambda t:self.reset())
        pass
    def bindMotor(self, motor:Motor):
        """绑定Motor和Encoder, 多个Motor可以绑定一个Encoder"""
        motor.encoder = self
        return self
    def readSpeed(self, t=None):  # 获取最新的counterRecord，可用作外部的回调函数
        return self.counterRecord
    def readPosition(self, t=None):
        return self.counterPosition
    def setZero(self):
        self.counterPosition = 0
        return self.counterPosition
    def setMax(self):
        __PositionLimit = self.readSpeed()
        config['PositionLimit'] = __PositionLimit
        writeConfig(config)
        return __PositionLimit
    def reset(self):
        self.counterRecord = self.counter
        self.counter = 0

class AbsoluteEncoder:
    """http://www.buruiter.com/col.jsp?id=116
    ModBus RS485
    若使用无线RS485传输模组，将查询间隙调至500左右。
    """
    def __init__(self, UARTid:int=2,UARTpinNums=(17, 16), updatePeriod:int=100, speedGaugeTimerPeriod=1000, WritePositionTimerNum:int=0, speedGaugeTimerNum:int=2, positionOffset:int=0) -> None:
        self.ModBus = ModbusRTUMaster(
            uart_id=UARTid,
            baudrate=9600,          # optional, default 9600
            data_bits=8,            # optional, default 7
            stop_bits=1,            # optional, default 1
            parity=None,            # optional, default None
            pins=UARTpinNums
        )
        self.position:int = 0
        self.positionOffset = positionOffset
        self.previousPositionStorage:int = 0
        self.speedGauge:int = 0
        self.WritePositionTimer = Timer(WritePositionTimerNum)
        self.WritePositionTimer.init(mode=Timer.PERIODIC, period=updatePeriod, callback=lambda t: self.__WritePositionTimerCallback())
        self.speedGaugeTimer = Timer(speedGaugeTimerNum)
        self.speedGaugeTimer.init(mode=Timer.PERIODIC, period=speedGaugeTimerPeriod, callback=lambda t: self.__SpeedGaugeTimerCallback())
    def __WritePositionTimerCallback(self):
        try:
            register_value = self.ModBus.read_holding_registers(
                slave_addr=1,
                starting_addr=0,
                register_qty=1,
                signed=False)
            if config["Debug_Encoder"]:
                print(register_value)
            self.position = -register_value[0] + self.positionOffset
        except Exception as e:
            print("Encoder callback error", e)
    def __SpeedGaugeTimerCallback(self):
        self.speedGauge = self.position - self.previousPositionStorage
        self.previousPositionStorage = self.position


def setMotorPinFromDirection(direction:-1|0|1|None):
    """根据direction设定方向相关针脚电平"""
    if direction == None: # 滑动， 不使用
        return [1, 1]
    else:
        mappingDict = {
            -1: (0, 1), # 正传 -> 收回
            0: (0, 0), # 制动
            1: (1, 0)  # 反转 -> 伸出
        }
        return mappingDict[direction]


async def controlTapeAWithPID(driver:PWMMotorDriver, encoder:AbsoluteEncoder, pid:PID):
    print("Handing control to PID")
    duty=0
    pid.output_limits = (0, 1000)
    while True:
        try:
            position = encoder.position
            pidMax = 512 +  400 * position / 20000
            pidMin = -612 + 800 * position / 20000
            if pidMax < 512:
                pidMax = 512
            if pidMax > 1023:
                pidMax = 1023
            if pidMin < -100:
                pidMin = -100
            pid.output_limits = (int(pidMin), int(pidMax))
            power = pid(position)
            duty = abs(power)
            duty = int(duty)
            if power > 0:
                direction = 1
            else:
                direction = -1
            try:
                driver.updateMotor(driver.Motor1, direction, duty) 
                if driver.Motor2:
                    driver.updateMotor(driver.Motor2, direction, duty)
            except Exception as e:
                print(e, duty)
                driver.updateMotor(driver.Motor1, 0, 0)
                if driver.Motor2:
                    driver.updateMotor(driver.Motor2, 0, 0)
            await asyncio.sleep(0)
        except Exception as e:
            print("PID Exception", e) # 有可能因无线485传输失败，一旦错误进行抱死。
            driver.updateMotor(driver.Motor1, 0, 0) 
            if driver.Motor2:
                driver.updateMotor(driver.Motor2, 0, 0)

async def controlTapeBWithPID(driver:PWMMotorDriver, encoder:AbsoluteEncoder, pid:PID):
    print("Handing control to PID")
    duty=0
    pid.output_limits = (0, 1000)
    start_time = time.time()
    last_time = start_time
    while True:
        try:
            position = encoder.position
            pidMax = 512 +  511 * position / 20000
            pidMin = -712 + 800 * position / 20000
            if pidMax < 512:
                pidMax = 512
            if pidMax > 1023:
                pidMax = 1023
            if pidMin < -300:
                pidMin = -300
            if position < 3000:
                pidMax = 200
            pid.output_limits = (int(pidMin), int(pidMax))
            current_time = time.time()
            power = pid(position)
            duty = abs(power)
            duty = int(duty)
            if power > 0:
                direction = 1
            else:
                direction = -1
            try:
                if pid.auto_mode == True:
                    driver.updateMotor(driver.Motor1, direction, duty)
            except Exception as e:
                print(e, duty)
            await asyncio.sleep(0)
        except Exception as e:
            print("PID Exception", e)
    
