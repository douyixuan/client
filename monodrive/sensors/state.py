
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

#import matplotlib
import numpy as np
import struct
#import matplotlib.patches as patches

from . import BaseSensor
#matplotlib.use('TkAgg')


class State(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(State, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):

        data_dict = {
            'time_stamp': time_stamp,
            'game_time': game_time
        }
        return data_dict



