
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

#import matplotlib
import numpy as np
import struct
#import matplotlib.patches as patches

from . import BaseSensor
import json
#matplotlib.use('TkAgg')

trajectory_data = []

class State(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(State, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        payload = frame.decode('utf-8')
        state = json.loads(payload)
        state['time'] = time_stamp
        state['game_time'] = game_time
        trajectory_data.append(state)

        text_file = open("Trajectory.json", "w")
        text_file.write(json.dumps(trajectory_data))
        text_file.close()

        return state



