__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseVehicle

# for keyboard control
import threading
import time

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class OpenSenseVehicle(BaseVehicle):
    def __init__(self, simulator_config, vehicle_config, restart_event=None, road_map=None, **kwargs):
        super(OpenSenseVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.throttle = 0.0
        self.steer = 0.0


    def drive(self, sensors):
        # logging.getLogger("control").debug("Control Forward,Steer = {0},{1}".format(self.throttle,self.steer))
        self.stop_vehicle()
        control = {'forward': 0.0, 'right': 0.0}
        return control

    def stop(self):
        self.keyboard_thread_running = False
        super(OpenSenseVehicle, self).stop()
        self.keyboard_thread.join()