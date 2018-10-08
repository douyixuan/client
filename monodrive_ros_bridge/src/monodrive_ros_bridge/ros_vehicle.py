from ackermann_msgs.msg import AckermannDrive
import math
import rospy

from monodrive.vehicles import TeleportVehicle
from monodrive.networking import messaging


def clamp(min_value, n, max_value):
    return max(min_value, min(n, max_value))


class RosVehicle(TeleportVehicle):
    def __init__(self, simulator, vehicle_config, map_data=None, restart_event=None, **kwargs):
        super(RosVehicle, self).__init__(simulator, vehicle_config, map_data, restart_event)

        self.gps_sensor = None
        for sensor in self.sensors:
            if sensor.type == 'GPS':
                self.gps_sensor = sensor
                break

        rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.process_ackermann_message)
        self.ackermann_publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

    # def drive(self, sensors):
    #     for sensor in sensors:
    #         if sensor.type == 'GPS':
    #             self.gps_state = sensor.get_message(block=True, timeout=0.0)
    #
    #     return super(RosVehicle, self).drive(sensors)

    def step(self, control):
        # convert to ackermann message
        max_angle = math.radians(40.0)
        max_speed = 27 # m/s
        angle = clamp(-max_angle, max_angle * self.steer * -1, max_angle) # ackermann left is positive
        angle_velocity = 0.0
        speed = clamp(-max_speed, max_speed * self.throttle, max_speed)
        accel = 0.0
        jerk = 0.0
        message = AckermannDrive(steering_angle=angle, steering_angle_velocity=angle_velocity,
                                 speed=speed, acceleration=accel, jerk=jerk)
        self.ackermann_publisher.publish(message)

    def process_ackermann_message(self, message):
        # convert to monodrive control command
        angle = -message.steering_angle / math.radians(40.0) # ackermann left is positive
        speed = message.speed / 27
        self.send_control_data({
            'forward': speed,
            'right': angle
        })

    def send_control_data(self, control_data):
        forward = control_data['forward']
        right = control_data['right']
        rospy.loginfo("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = self.simulator.request(msg)
        if resp is None:
            rospy.logerr(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))

