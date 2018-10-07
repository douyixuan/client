from multiprocessing import Event
import rospy

from monodrive.vehicles import TeleportVehicle


class SensorDataMonitor:

    def __init__(self, sensor):
        self.sensor = sensor
        rospy.Subscriber("vehicle_control", String, callback)


class RosVehicle(TeleportVehicle):
    def __init__(self, simulator, vehicle_config, map_data=None, restart_event=None, **kwargs):
        super(RosVehicle, self).__init__(simulator, vehicle_config, map_data, restart_event)
        self.running = False
        self.episode_event = Event()


    def step(self, control):
        pass

    # def start(self):
    #     rospy.loginfo("--> starting vehicle process")
    #     #self.running = True
    #     #super(RosVehicle, self).start()
    #     [p.start() for p in self.get_process_list()]
    #
    #     rospy.loginfo("start streaming sensors")
    #     self.active_sensors = []
    #     for sensor in self.sensors:
    #         response = sensor.send_start_stream_command(self.simulator)
    #         if response and response['success']:
    #             self.active_sensors.append(sensor)
    #
    #     rospy.loginfo("waiting for sensors ready")
    #     [s.wait_until_ready() for s in self.active_sensors]
    #
    #     rospy.loginfo("starting vehicle loop")
    #     # Kicks off simulator for stepping
    #     self.init_vehicle_loop()
    #
    #     rospy.loginfo("<-- vehicle process started")
    #
    #
    # def stop(self):
    #     rospy.loginfo("stopping vehicle process")
    #     self.stop_vehicle()
    #
    #     rospy.loginfo("stopping all sensors")
    #
    #     [s.send_stop_stream_command(self.simulator) for s in self.active_sensors]
    #
    #     rospy.loginfo("stopping sensor processes")
    #     [s.stop() for s in self.sensors]
    #     [s.join() for s in self.sensors]
    #
    #     rospy.loginfo("sensor termination complete")


    def end_episode(self):
        self.episode_event.set()