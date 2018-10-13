#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



"""
Classes to handle mono sensors
"""
import math
import numpy as np
import tf
import rospy
import traceback

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped, Vector3
from sensor_msgs.msg import CameraInfo, Imu, NavSatFix
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from velodyne_msgs.msg import VelodynePacket, VelodyneScan


from monodrive.transform import Transform as mono_Transform, Translation as mono_Translation, Rotation as mono_Rotation
from monodrive_ros_bridge.transforms import mono_transform_to_ros_transform, ros_transform_to_pose
from monodrive_ros_bridge.msg import BoundingBox, LaneInfo, Rpm, Target, Waypoint


cv_bridge = CvBridge(
)  # global cv bridge to convert image between opencv and monodrive_ros_bridge


class SensorHandler(object):
    """
    Generic Sensor Handler

    A sensor handler compute the associated monodrive_ros_bridge message to a mono sensor data, and the transform asssociated to the
    sensor.
    These messages are passed to a *process_msg_fun* which will take care of publishing them.
    """
    _count = 700

    def __init__(self,
                 name,
                 sensor=None,
                 process_msg_fun=None):
        """
        :param name: sensor name
        :param sensor: monodrive sensor instance
        :param mono_settings: mono_settings object
        :param process_msg_fun: function to call on each new computed message
        """
        self._instance_id = SensorHandler._count
        SensorHandler._count = SensorHandler._count + 1

        self.process_msg_fun = process_msg_fun
        self.sensor = sensor
        self.name = name
        self.parent_frame_id = "ego"
        self.frame_id = self.sensor.type + name

    def process_sensor_data(self, vehicle, cur_time):
        """
        process a mono sensor data object

        Generate sensor message and transform message

        :param data: mono sensor data
        :param cur_time: current monodrive_ros_bridge simulation time
        :return:
        """
        try:
            data = self.sensor.get_display_message(block=True, timeout=1.0)
            if data:
                self._compute_sensor_msg(data, cur_time)
                self._compute_transform(data, self._get_transform(vehicle), cur_time)
        except Exception as e:
            traceback.print_exc()
            rospy.loginfo(e)

    def calculate_rotation(selfs, forward):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(np.append(forward, [0.0]))
        return mono_Rotation(0, yaw, 0)

    def _get_transform(self, vehicle):
        position = vehicle.gps_sensor.world_location if vehicle.gps_sensor else None
        forward = vehicle.gps_sensor.forward_vector if vehicle.gps_sensor else None
        if forward is not None:
            rotation = self.calculate_rotation(forward)
        else:
            rotation = None

        if position is not None and rotation is not None:
            transform = mono_Transform(mono_Translation(position[0], position[1], position[2]),
                                  rotation)
            x, y, z = tf.transformations.translation_from_matrix(transform.matrix)
            return transform
        elif position:
            return mono_Transform(mono_Translation(position[0], position[1], position[2]))
        elif rotation:
            return mono_Transform(rotation)

        return None

    def _compute_sensor_msg(self, data, cur_time):
        """
        Compute the monodrive_ros_bridge msg associated to mono data
        :param data: SensorData object
        :param cur_time: current monodrive_ros_bridge simulation time
        """
        raise NotImplemented

    def _compute_transform(self, data, ref, cur_time):
        """
        Compute the tf msg associated to mono data

        :param data: SensorData object
        :param cur_time: current monodrive_ros_bridge simulation time
        """
        raise NotImplemented

    def get_marker(self, header, transform, type, color, size):
        marker = Marker(header=header)
        marker.id = self._instance_id
        marker.text = "id = {}".format(marker.id)
        #marker.action = Marker.ADD
        marker.color = color
        ros_transform = mono_transform_to_ros_transform(
            # mono_Transform(object.bounding_box.transform) *
            transform)
        marker.pose = ros_transform_to_pose(ros_transform)

        marker.scale = size

        marker.type = type
        return marker


class LidarHandler(SensorHandler):
    """
    Class to handle Lidar sensors
    """

    def __init__(self, name, sensor, **kwargs):
        super(LidarHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        topic = 'velodyne_packets'

        scan = []
        for lidar_packet in sensor_data:
            new_sensor_data = bytes(lidar_packet)
            scan.append(VelodynePacket(stamp=cur_time, data=new_sensor_data))

        if len(scan):
            header = Header()
            header.stamp = cur_time
            header.frame_id = self.frame_id

            self.process_msg_fun(topic, VelodyneScan(header=header, packets=scan))

    def _compute_transform(self, sensor_data, ref, cur_time):
        if ref is None:
            return

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id
        t.transform = mono_transform_to_ros_transform(
#            mono_Transform(mono_Translation(0,0,0))
             self.sensor.get_transform()
        )

        # for some reasons lidar sends already rotated cloud,
        # so it is need to ignore pitch and roll
        r = t.transform.rotation
        quat = [r.x, r.y, r.z, r.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # if ref:
        #     ref = mono_transform_to_ros_transform(ref)
        #     t.transform.translation.x = ref.translation.x + t.transform.translation.x
        #     t.transform.translation.y = ref.translation.y + t.transform.translation.y
        #     t.transform.translation.z = ref.translation.z + t.transform.translation.z
        # t.transform.translation.x = 0
        # t.transform.translation.y = 0
        # t.transform.translation.z = 0
        # if ref:
        #     rospy.loginfo("lidar tf: {0}".format(t.transform))
        #     rospy.loginfo("ego   tf: {0}".format(mono_transform_to_ros_transform(ref)))
        self.process_msg_fun('tf', t)

        self.process_msg_fun('sensors', self.get_marker(t.header,
                                                        self.sensor.get_transform(),
                                                        Marker.ARROW,
                                                        ColorRGBA(0.9,0.5,0,1),
                                                        Vector3(1.5,0.2,0.2)))


class CameraHandler(SensorHandler):
    """
    Class to handle Camera sensors
    """

    def __init__(self, name, sensor, **kwargs):
        super(CameraHandler, self).__init__(
            name, sensor=sensor, **kwargs)

        self.topic_image = '/'.join([self.frame_id, 'image_raw'])
        self.topic_cam_info = '/'.join([self.frame_id, 'camera_info'])
        self.build_camera_info()

    def build_camera_info(self):
        """
        computing camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        '''camera_info.width = self.mono_object.ImageSizeX
        camera_info.height = self.mono_object.ImageSizeY
        camera_info.distortion_model = 'plumb_bob'
        cx = self.mono_object.ImageSizeX / 2.0
        cy = self.mono_object.ImageSizeY / 2.0
        fx = self.mono_object.ImageSizeX / (
            2.0 * math.tan(self.mono_object.FOV * math.pi / 360.0))
        '''

        #TODO FIX THIS TO READ FROM THE VEHICLE_CONFIG
        camera_info.width = self.sensor.width
        camera_info.height = self.sensor.height
        camera_info.distortion_model = 'plumb_bob'
        cx = self.sensor.width / 2
        cy = self.sensor.height / 2
        fx = cx / (math.tan(self.sensor.get_fov() * math.pi / 360.0))


        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        self._camera_info = camera_info

    def _compute_sensor_msg(self, sensor_data, cur_time):
        encoding = 'bgra8'

        #data = pickle.loads(sensor_data)
        data = np.array(bytearray(sensor_data['image']), dtype=np.uint8).reshape(self.sensor.height, self.sensor.width, 4)

        img_msg = cv_bridge.cv2_to_imgmsg(data, encoding=encoding)
        img_msg.header.frame_id = self.frame_id
        img_msg.header.stamp = cur_time

        cam_info = self._camera_info
        cam_info.header = img_msg.header

        self.process_msg_fun(self.topic_cam_info, cam_info)
        self.process_msg_fun(self.topic_image, img_msg)

    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        # for camera we reorient it to look at the same axis as the opencv projection
        # in order to get easy depth cloud for RGBD camera
        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)

        self.process_msg_fun('sensors', self.get_marker(t.header,
                                                        self.sensor.get_transform(),
                                                        Marker.ARROW,
                                                        ColorRGBA(0.5,0.9,0.5,0),
                                                        Vector3(0.5,0.1,0.1)))


class ImuHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(ImuHandler, self).__init__(
            name, sensor=sensor, **kwargs)
        self.seq = 0

    def _compute_sensor_msg(self, sensor_data, cur_time):
        self.seq += 1

        header = Header()
        header.stamp = cur_time
        header.frame_id = self.frame_id
        header.seq = self.seq

        msg = Imu()
        msg.header = header
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 0
        msg.orientation_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        msg.angular_velocity.x = sensor_data['angular_velocity_vector'][0]
        msg.angular_velocity.y = sensor_data['angular_velocity_vector'][1]
        msg.angular_velocity.z = sensor_data['angular_velocity_vector'][2]
        msg.angular_velocity_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        msg.linear_acceleration.x = sensor_data['acceleration_vector'][0]
        msg.linear_acceleration.y = sensor_data['acceleration_vector'][1]
        msg.linear_acceleration.z = sensor_data['acceleration_vector'][2]
        msg.linear_acceleration_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        self.process_msg_fun('imu', msg)


    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)

        self.process_msg_fun('sensors', self.get_marker(t.header,
                                                        self.sensor.get_transform(),
                                                        Marker.SPHERE,
                                                        ColorRGBA(0.5,0.9,0,0.5),
                                                        Vector3(0.2,0.2,0.2)))


class GpsHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(GpsHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.frame_id

        msg = NavSatFix()
        msg.header = header

        msg.status.status = 0
        msg.status.service = 1
        msg.latitude = sensor_data['lat']
        msg.longitude = sensor_data['lng']
        msg.altitude = sensor_data['elevation']

        # todo, add covariance
        msg.position_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
        msg.position_covariance_type = 0  # unknown

        self.process_msg_fun('gps', msg)


    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)

        self.process_msg_fun('sensors', self.get_marker(t.header,
                                                        self.sensor.get_transform(),
                                                        Marker.SPHERE,
                                                        ColorRGBA(0.5,0,0.5,0.9),
                                                        Vector3(0.2,0.2,0.2)))


class RpmHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(RpmHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.frame_id

        msg = Rpm()
        msg.header = header
        msg.wheel_number = sensor_data['wheel_number']
        msg.wheel_rpm = sensor_data['wheel_rpm']

        self.process_msg_fun('rpm', msg)


    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform = mono_transform_to_ros_transform(
            self.base_transform *
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)

        self.process_msg_fun('sensors', self.get_marker(t.header,
                                                        self.sensor.get_transform(),
                                                        ColorRGBA(0.5,0,0.9,0.5),
                                                        Vector3(1.5,0.2,0.2)))



class BoundingBoxHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(BoundingBoxHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.frame_id

        msg = BoundingBox(header=header, targets=[])
        for i in range(0, len(sensor_data['distances'])):
            msg.targets.append(Target(id=i, distance=sensor_data['distances'][i], angle=sensor_data['angles'][i],
                                      relative_rotation=-sensor_data['box_rotations'][i]+90,
                                      velocity=sensor_data['velocities'][i],
                                      #in_radar_fov=sensor_data['radar_distances'][i],
                                      center=Point(x=sensor_data['x_points'][i], y=sensor_data['y_points'][i]),
                                      extent=Point(x=sensor_data['x_bounds'][i], y=sensor_data['y_bounds'][i],
                                                   z=sensor_data['z_bounds'][i])))

        self.process_msg_fun('boundingbox', msg)

    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        # for some reasons lidar sends already rotated cloud,
        # so it is need to ignore pitch and roll
        r = t.transform.rotation
        quat = [r.x, r.y, r.z, r.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)


class WaypointHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(WaypointHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.frame_id

        msg = Waypoint(header=header, current_lane=sensor_data['lane_number'], lanes=[])
        for i in range(0, len(sensor_data['points_by_lane'])):
            lane = LaneInfo(lane_id=i, speed_limit=sensor_data['speed_limit_by_lane'][i], points=[])
            for x, y in sensor_data['points_by_lane'][i]:
                lane.points.append(Point(x=x, y=y))

            msg.lanes.append(lane)

        self.process_msg_fun('waypoint', msg)

    def _compute_transform(self, sensor_data, ref, cur_time):

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)
