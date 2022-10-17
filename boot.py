import main_Hub, main_TapeA_Winch, main_TapeB_Drum
from config import loadConfig, writeConfig
import uasyncio as asyncio


config = loadConfig()
if config["Core_Name"] == "Hub":
    print("Starting Hub!")
    asyncio.run(main_Hub.main())


if config["Core_Name"] == "main_TapeA_Winch":
    print("Starting Hub!")
    asyncio.run(main_TapeA_Winch.main())

if config["Core_Name"] == "main_TapeB_Drum":
    print("Starting Hub!")
    asyncio.run(main_TapeB_Drum.main())