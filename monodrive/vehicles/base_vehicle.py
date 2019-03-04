import logging
import time
import threading
try:
    import cPickle as pickle
except:
    import _pickle as pickle

try:
    import queue as PythonQueue
except ImportError:
    import Queue as PythonQueue

import sys, traceback

from monodrive.networking import messaging
from monodrive.constants import ClockMode_AutoStep, ClockMode_ClientStep

import multiprocessing


class BaseVehicle(object):
    def __init__(self, simulator_config, vehicle_config, restart_event=None, **kwargs):
        super(BaseVehicle, self).__init__()
        self.client = None
        self.simulator_config = simulator_config
        self.name = vehicle_config.id
        self.sensors = []
        self.restart_event = restart_event
        self.previous_control_sent_time = None
        self.control_thread = None
        self.b_control_thread_running = True

        #FROM old sensor manager
        self.vehicle_config = vehicle_config
        self.sensor_process_dict = {}
        self.init_sensors()

        self.vehicle_update_rate = 10 # ticks per second
        self.vehicle_stop = multiprocessing.Event()
        self.vehicle_thread = None
        self.vehicle_step_event = multiprocessing.Event()

    def init_vehicle_loop(self, client):
        self.vehicle_thread = threading.Thread(target=self.vehicle_loop(client))
        self.vehicle_thread.daemon = False
        self.vehicle_thread.start()

    def stop_vehicle(self, timeout=2):
        self.vehicle_stop.set()
        if self.vehicle_thread:
            self.vehicle_thread.join(timeout=timeout)
        self.vehicle_thread = None
    
    def vehicle_loop(self, client):
        # step the vehicle to start the measurements
        #self.step(client, {'forward': 0.0, 'right': 0.0})
        
        sensors = self.get_sensors()
        #time.sleep(5)
        # self.ready_event.set()
        while not self.vehicle_stop.wait(.25):
            #start_time = time.time()
            control = self.drive(sensors)
            self.step2(client, control)
            #print("vehicle_loop time = " + str(time.time() - start_time))

    def step2(self, client, control_data):
        #start_time = time.time();
        forward = control_data['forward']
        right = control_data['right']
        logging.getLogger("control").debug("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = client.request(msg)
        #print("step time = " + str(time.time()-start_time))

        if resp is None:
            logging.getLogger("control").error(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))

    def step(self, client, control_data):
        self.control_thread = threading.Thread(target=self.do_control_thread(client, control_data))
        self.control_thread.start()

    def do_control_thread(self, client, control_data):
        forward = control_data['forward']
        right = control_data['right']
        logging.getLogger("control").debug("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = client.request(msg)
        if resp is None:
            logging.getLogger("control").error(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))

    def drive(self, sensors):
        raise NotImplementedError("To be implemented in base class")

    def get_sensor(self, sensor_type, id):
        for sensor in self.sensors:
            if sensor.type == sensor_type and sensor.sensor_id == id:
                return sensor
        return None

    def get_sensors(self):
        return self.sensors

    def init_sensors(self):
        for sensor_config in self.vehicle_config.sensor_configuration:
            if not sensor_config['sensor_process']:
                continue
            sensor_instance = self.init_sensor(sensor_config)
            self.sensor_process_dict[sensor_instance.name] = sensor_instance
            self.sensors.append(sensor_instance) 

    def init_sensor(self, sensor_config):
        sensor_type = sensor_config['type']
        _Sensor_Class = self.vehicle_config.get_class(sensor_type)

        sensor_instance = _Sensor_Class(sensor_type, sensor_config, self.simulator_config)
        return sensor_instance

    def get_process_list(self):
        _processes = []
        for sensor in self.sensor_process_dict.values():
            _processes.append(sensor)
            
            if getattr(sensor, 'packetizer_process', None) is not None:
                _processes.append(sensor.packetizer_process)
        return _processes

    def start_sensor_streaming(self, client):
        [s.send_start_stream_command(client) for s in self.sensors]
        return 1

    def stop_sensor_streaming(self, client):
        [s.stop_sensor_command(client) for s in self.sensors]
        return 1

    def start_sensor_listening(self):
        [p.start() for p in self.get_process_list()]
        logging.getLogger("vehicle").debug("waiting for sensors ready")
        [s.wait_until_ready() for s in self.sensors]
        logging.getLogger("vehicle").debug("all sensors are ready")
        return 1

    '''def start(self):
        [p.start() for p in self.get_process_list()]

        logging.getLogger("vehicle").debug("start streaming sensors")
        #[s.send_start_stream_command(self.simulator) for s in self.sensors]
        [s.send_start_stream_command(self.client) for s in self.sensors]

        logging.getLogger("vehicle").debug("waiting for sensors ready")
        [s.wait_until_ready() for s in self.sensors]

        logging.getLogger("vehicle").info("starting vehicle loop")
        # Kicks off simulator for stepping
        self.init_vehicle_loop()'''


    def stop(self):
        logging.getLogger("sensor").info("stopping vehicle")
        self.stop_vehicle()
        logging.getLogger("sensor").info("stopping all sensors")

        # stopping simulator from sending data
        #logging.getLogger("sensor").debug("stopping sensor from streaming data")
        #[s.send_stop_stream_command(self.simulator) for s in self.sensors]

        logging.getLogger("sensor").info("stopping sensor processes")
        [s.stop() for s in self.sensors]

        # self.print_all_stacktraces()
        logging.getLogger("sensor").debug("joining sensor processes STARTED")
        [s.join() for s in self.sensors]
        logging.getLogger("sensor").debug("joining sensor processes COMPLETE")

        logging.getLogger("sensor").info("sensor termination complete")

    def print_all_stacktraces(self):
        print("\n*** STACKTRACE - START ***\n")
        code = []
        for threadId, stack in sys._current_frames().items():
            threadName = ''
            for t in threading.enumerate():
                if t.ident == threadId:
                    threadName = t.name
            code.append("\n# ThreadID: %s %s" % (threadId, threadName))
            for filename, lineno, name, line in traceback.extract_stack(stack):
                code.append('File: "%s", line %d, in %s' % (filename,
                                                            lineno, name))
                if line:
                    code.append("  %s" % (line.strip()))
        for line in code:
            print(line)
        print("\n*** STACKTRACE - END ***\n")

    # example to push generated fmcw(ndarray) to sensor config
    # the fmcw can be generated from anywhere - using an example of a pre-generated one here
    def update_fmcw_in_config(self):
        radar_sensor = [s for s in self.sensors if s.type == 'Radar'][0]
        self.vehicle_config.update_radar_waveform(radar_sensor.tx_waveform)


