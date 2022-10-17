from machine import Pin, Timer, reset
from utility import PWMMotorDriver, Motor, WiFi, controlTapeBWithPID, AbsoluteEncoder
from pinNums import TapeB_Drum
from microdot_asyncio import Microdot
from config import loadConfig, writeConfig
import uasyncio as asyncio
from PID import PID
import utime as time

config = loadConfig()

# Miscellaneous
debugging = config["Debugging"]
Debug_SkipNetwork = config["Debug_SkipNetwork"]
Debug_DisablePID = config["Debug_DisablePID"]
speedGaugeTimerPeriod = 500
# Initialize System
## Get Pin Nums
pwmPin1Num = TapeB_Drum["pwmPin1Num"]
motorPinA1Num = TapeB_Drum["motorPinA1Num"]
motorPinB1Num = TapeB_Drum["motorPinB1Num"]

## Construct Pin Object
pwmPin1 = Pin(pwmPin1Num)
motorPinA1 = Pin(motorPinA1Num, Pin.OUT, Pin.PULL_DOWN)
motorPinB1 = Pin(motorPinB1Num, Pin.OUT, Pin.PULL_DOWN)

# Initialize Core Modules
## Mechanical Modules
motor1 = Motor(pwmPin1, motorPinA1, motorPinB1)
driver1 = PWMMotorDriver(motor1, None)
driver1.updateMotor(motor1, 0, 0)
encoder1 = AbsoluteEncoder(2, (17, 16), 100, 1000, 0, 2, 24000)
time.sleep(5)
pid = PID(-0.5, -0.001, 0.1, setpoint=encoder1.position, sample_time = 0.01)
print("PID Initialized")

## Web Interface
if Debug_SkipNetwork == False:
    onboardWiFi = WiFi()
    onboardWiFi.connect(config["Network_SSID"], config["Network_Password"])
    webapp = Microdot()

    @webapp.route('/ping')
    def ping(request):
        return config, 200, {'Content-Type': 'application/json'}

    @webapp.route('/updateConfig')
    def updateConfig(request):
        """Update config and restart

        Args:
            configKey
            newValue
            toReset:int
        """
        config[request.args["configKey"]] =  request.args["newValue"]
        writeConfig(config)
        if int(request.args["toReset"]) == 1:
            reset()

    @webapp.route('/manualControl')
    def manualControl(request):
        """_summary_

        Args:
            controlType: ["Position", "Distance"]
            value,
            duty
        """
        forceRun = False
        controlType = request.args["controlType"]
        value = int(request.args["value"])
        duty = int(request.args["duty"])
        forceRun = bool(request.args["forceRun"])
        print("Manually Controlling, Type:%s, Value:%s, Duty:%s" % (controlType, value, duty))
        if controlType == "Position":
            if value not in (range(0, config["PositionLimit"])):
                return "Position Illegal", 200, {'Content-Type': 'text/plain'}
            driver1.updateMotorToPosition(driver1.Motor1, value, accelerationSpan=1000, duty=duty)
        if controlType == "Distance":
            driver1.updateMotorByDistance(driver1.Motor1, value, accelerationSpan=1000, duty=duty, forceRun=forceRun)
        while driver1.Motor1.encoder.readSpeed() !=0:
            pass
        response = driver1.Motor1.encoder.readPosition()
        return str(response), 200, {'Content-Type': 'text/plain'}

    @webapp.route('/setPIDPosition')
    def setPIDPosition(request):
        position = int(request.args["position"])
        pid.setpoint = position
        return


# Main Function
async def main():
    await asyncio.sleep(5)
    # Debugging Code Block
    speedGaugeTimer2=Timer(1)
    speedGaugeTimer2.init(mode=Timer.PERIODIC, period=3000,callback=lambda t:print(
        "Encoder", encoder1.speedGauge, encoder1.position,
        "PID", pid.setpoint, driver1.Motor1.PWMObj.duty(), pid.output_limits,
        "MotorPin",driver1.Motor1.motorPinA.value(),driver1.Motor1.motorPinB.value()))
    # Actual Function
    asyncio.create_task(webapp.start_server(port=80))
    asyncio.create_task(controlTapeBWithPID(driver1, encoder1, pid))
    while True:
        await asyncio.sleep(0)






