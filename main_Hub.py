from machine import Pin, reset
import time
from utility import WiFi
import pinNums
from config import loadConfig
import random
import urequests as requests

triggerPin = Pin(pinNums.Hub["triggerPinNum"], Pin.IN, Pin.PULL_UP)
next_call = 0
lastZeroMinute = time.localtime(time.time())[4]
triggerableFlag = False
config = loadConfig()
Debug_SkipNetwork = config["Debug_SkipNetwork"]



if Debug_SkipNetwork == False:
    onboardWiFi = WiFi()
    onboardWiFi.connect(config["Network_SSID"], config["Network_Password"])

# Main Function
def main():
    global next_call
    global triggerableFlag
    print("New Start! Starting!")
    try:
        nowMinute = time.localtime(time.time())[4]
        __InitialSleepTimer = 30
        if config['Debugging']:
            print("Debugging. Initial Sleep Timer skipped to 20s")
            __InitialSleepTimer = 10
        while __InitialSleepTimer > 0:
            print(f"Initial Sleeping, {__InitialSleepTimer}s remaining")
            __InitialSleepTimer -= 5
            time.sleep(5)
    except Exception as e:
        print("Initial Sleep failed, resetting...")
        reset()
        time.sleep(5)
    print("Initial Sleep Finished")
    lastZeroMinute = nowMinute
    triggerableFlag = True
    def triggerPoller(triggerPin:Pin):
        global triggerableFlag
        global lastZeroMinute
        while True:
            time.sleep(5)
            print("New Polling Interval")
            if triggerableFlag == False:
                print("Sleeping due to triggerableFlag")
                time.sleep(5)
                continue
            triggerPinValue = triggerPin.value()
            if triggerPinValue == 1:
                print("Polled as Inactive")
                continue
            print("Successful Trigger! Making Requests!")
            triggerableFlag = False
            nowMinute = time.localtime(time.time())[4]
            ATarget = random.randrange(1000, 18000,100)
            BTarget = random.randrange(3000, 18000,100)
            try:
                print(f"Tape A Going to {ATarget}")
                requests.get(f'http://10.168.1.228/setPIDPosition?position={ATarget}')
            except Exception as e:
                print("Failed! Resetting! Error:", e)
                reset()
            try:
                print(f"Tape B Going to {BTarget}")
                requests.get(f'http://10.168.1.231/setPIDPosition?position={BTarget}')
            except Exception as e:
                print("Failed! Resetting! Error:", e)
                reset()  
            print("Successful Motion. Re-enabling triggerableFlag in 60s")
            __ReEnablingTimer = 60
            while __ReEnablingTimer > 0:
                print(f"Sleeping until re-enabling, {__ReEnablingTimer}s remaining.")
                __ReEnablingTimer -= 5
                time.sleep(5)
            print("Reenabling triggerableFlag")
            triggerableFlag = True
    triggerPoller(triggerPin)