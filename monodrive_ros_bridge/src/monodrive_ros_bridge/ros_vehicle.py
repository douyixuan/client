from ackermann_msgs.msg import AckermannDrive
import math
import rospy

from monodrive.vehicles import BaseVehicle
from monodrive.networking import messaging


def clamp(min_value, n, max_value):
    return max(min_value, min(n, max_value))


class RosVehicle(BaseVehicle):
    def __init__(self, simulator, vehicle_config, map_data=None, restart_event=None, **kwargs):
        super(RosVehicle, self).__init__(simulator, vehicle_config, map_data, restart_event)

        self.throttle = 0.0
        self.steer = 0.0
        self.gps_sensor = None

        for sensor in self.sensors:
            if sensor.type == 'GPS':
                self.gps_sensor = sensor
                break

        if self.gps_sensor is None:
            rospy.loginfo("NO GPS SENSOR FOUND")

        rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.process_ackermann_message)
        #self.ackermann_publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

    def vehicle_loop(self):
        # step the vehicle to start the measurements
        self.step({'forward': 0.0, 'right': 0.0})

        while not self.vehicle_stop.wait(self.vehicle_update_rate):
            pass

    def step_episode(self):
        control = self.drive(self.sensors)
        self.step(control)
        sensor_data = {}
        for sensor in self.sensors:
            try:
                sensor_data[sensor.name] = sensor.get_display_message(block=True, timeout=1.0)
            except Exception as e:
                rospy.loginfo("no data for {0}, {1}".format(sensor.name, e))

        return sensor_data

    def drive(self, sensors):
        return {
            'forward': self.throttle,
            'right': self.steer
        }

    # def step(self, control):
    #     # convert to ackermann message
    #     max_angle = math.radians(40.0)
    #     max_speed = 27  # m/s
    #     angle = clamp(-max_angle,
    #                   max_angle * control.get('right', self.steer * -1),  # ackermann left is positive
    #                   max_angle)
    #     angle_velocity = 0.0
    #     speed = clamp(-max_speed,
    #                   max_speed * control.get('forward', self.throttle),
    #                   max_speed)
    #     accel = 0.0
    #     jerk = 0.0
    #     message = AckermannDrive(steering_angle=angle, steering_angle_velocity=angle_velocity,
    #                              speed=speed, acceleration=accel, jerk=jerk)
    #     self.ackermann_publisher.publish(message)

    def process_ackermann_message(self, message):
        # convert to monodrive control command
        rospy.loginfo("ackermann command: {0}".format(message))
        angle = -message.steering_angle / math.radians(40.0) # ackermann left is positive
        speed = message.speed / 27
        self.step({
            'forward': speed,
            'right': angle
        })

    def step(self, control_data):
        forward = control_data['forward']
        right = control_data['right']
#        rospy.loginfo("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = self.simulator.request(msg)
        if resp is None:
            rospy.logerr(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))

