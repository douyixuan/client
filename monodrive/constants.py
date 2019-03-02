
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import os


CONTROL_HEADER = 0x6d6f6e6f
RESPONSE_HEADER = 0x6f6e6f6d

STREAM_DATA_FORMAT_RAW = 0

# UUIDs for ingame manipulation
Status_ID               = u"Status_ID"
SimulatorConfig_ID      = u"SimulatorConfig_ID"
EgoVehicleConfig_ID     = u"EgoVehicleConfig_ID"
EgoControl_ID           = u"EgoControl_ID"
MapCommand_ID           = u"MapCommand_ID"
ScenarioConfig_ID       = u"ScenarioConfig_ID"
ScenarioInit_ID         = u"ScenarioInit_ID"
StreamData_ID           = u"StreamData_ID"
WaypointUpdate_ID       = u"WaypointUpdate_ID"
SpawnActorCommand_ID    = u"SpawnActorCommand_ID"
UpdateActorCommand_ID   = u"UpdateActorCommand_ID"
AttachSensorCommand_ID  = u"AttachSensorCommand_ID"
DetachSensorCommand_ID  = u"DetachSensorCommand_ID"
REPLAY_ConfigureTrajectoryCommand_ID    = u"REPLAY_ConfigureTrajectoryCommand_ID"
REPLAY_StepSimulationCommand_ID         = "REPLAY_StepSimulationCommand_ID"
REPLAY_ConfigureSensorsCommand_ID       = u"REPLAY_ConfigureSensorsCommand_ID"

VELOVIEW_PORT = 2368
VELOVIEW_IP = '127.0.0.1'

BASE_PATH = os.path.dirname(os.path.dirname(__file__))

ClockMode_Continuous = 0
ClockMode_AutoStep   = 1
ClockMode_ClientStep = 2
