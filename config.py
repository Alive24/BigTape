import json, copy
# Core

defaultConfigDict = {
    "Debugging": True,
    "Debug_SkipNetwork": True,
    "Debug_Encoder": True,
    "Core_Name": "Core_Name",
    "Debug_DisablePID": True,
    "Network_SSID": "TestSSID",
    "Network_Password": "TestPassword",
}


# Global Limit


def loadConfig():
    try:
        configFile = open("config.json",'r',encoding='utf-8')
        configDictToLoad = json.load(configFile)
        configDict = copy.deepcopy(defaultConfigDict)
        for key in list(configDictToLoad.keys()):
            configDict[key] = configDictToLoad[key]
        configFile.close()
    except Exception as e:
        print("loadConfig()错误")
        configDict = copy.deepcopy(defaultConfigDict)
        configFile = open("config.json",'w',encoding='utf-8')
        json.dump(configDict,configFile)
        configFile.close()
    return configDict


def writeConfig(configDict):
    try:
        configFile = open("config.json",'w',encoding='utf-8')
        json.dump(configDict,configFile)
        configFile.close()
    except Exception as e:
        print("writeConfig()错误")
        configFile = open("config.json",'w',encoding='utf-8')
        json.dump(configDict,configFile)
        configFile.close()
    loadConfig()
    return
