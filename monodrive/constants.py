
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import os


CONTROL_HEADER = 0x6d6f6e6f
RESPONSE_HEADER = 0x6f6e6f6d

STREAM_DATA_FORMAT_RAW = 0

# UUIDs for ingame manipulation
SIMULATOR_STATUS_UUID = u"2e1831fd-dd75-4ca3-bdc0-cfb8bbff9369"
STREAM_DATA_COMMAND_UUID = u"7fb9bacd-5d2b-4a68-8121-bdbd541a8136"
SIMULATOR_CONFIG_COMMAND_UUID = u"767e0f95-736f-4aee-bd65-38063aad5fc5"
VEHICLE_CONFIG_COMMAND_UUID = u"2f475af5-4412-4e7f-819a-4ccfea425285"
EGO_CONTROL_COMMAND_UID = u"25f6c0ac-07e0-420d-b363-3749f5fef52d"
WAYPOINT_UPDATE_COMMAND_UID = u"ed52b34f-63a6-46cc-9b74-c1cc1730431f"
SCENARIO_COMMAND_UID = u"a2655421-a3bf-424c-817e-c2046e95be30"
SCENARIO_INIT_COMMAND_UID = u"86f5c4b6-b959-4cd8-8382-e3bcb8cf7b4a"
MAP_COMMAND_UID = u"f1eba34c-894b-4fae-941f-46bf7faba0fd"
SPAWN_COMMAND_UID = u"e7a887bb-1649-401e-88c5-9d7ea8630ce5"
MOVE_ACTOR_COMMAND_UID = u"8b03d810-35c3-4083-bf84-faee126178ff"
VELOVIEW_PORT = 2368
#VELOVIEW_PORT = 11311

VELOVIEW_IP = '127.0.0.1'

BASE_PATH = os.path.dirname(os.path.dirname(__file__))

ClockMode_Continuous = 0
ClockMode_AutoStep   = 1
ClockMode_ClientStep = 2
