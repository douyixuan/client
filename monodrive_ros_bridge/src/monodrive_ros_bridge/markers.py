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
Classes to handle Agent object (player and non-player)
"""

from geometry_msgs.msg import TransformStamped, Transform
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker

from monodrive.transform import Transform as mono_Transform, Translation as mono_Translation, Rotation as mono_Rotation
from monodrive_ros_bridge.transforms import mono_transform_to_ros_transform, ros_transform_to_pose
import numpy as np
import tf

class AgentObjectHandler(object):
    """
    Generic class to convert mono agent information to monodrive_ros_bridge messages
    In monodrive_ros_bridge messages are represented as Marker message (thus they are viewable in Rviz).
    """

    def __init__(self, name, process_msg_fun=None, world_frame='map'):
        self.name = name
        self.world_frame = world_frame
        self.process_msg_fun = process_msg_fun
        self.lookup_table_marker_id = {}

    def process_msg(self, data, cur_time):
        """

        :param data: mono agent data
        :param cur_time: current monodrive_ros_bridge simulation time
        :return:
        """
        raise NotImplemented

    def get_marker_id(self, agent_id):
        """
        Return a unique marker id for each agent

        monodrive_ros_bridge marker id should be int32, mono/unrealengine seems to use int64
        A lookup table is used to remap agent_id to small number between 0 and max_int32

        :param agent_id:
        :return:
        """
        if agent_id not in self.lookup_table_marker_id:
            self.lookup_table_marker_id[agent_id] = len(
                self.lookup_table_marker_id)
        return self.lookup_table_marker_id[agent_id]


class PlayerAgentHandler(AgentObjectHandler):
    """
    Convert player agent into monodrive_ros_bridge message (as marker)
    """

    def __init__(self, name, **kwargs):
        super(PlayerAgentHandler, self).__init__(name, **kwargs)

    def get_transform(self, vehicle):
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


    def calculate_rotation(selfs, forward):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(np.append(forward, [0.0]))
        return mono_Rotation(0, yaw, 0)

    def process_msg(self, vehicle, cur_time):
        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.name

        data = self.get_transform(vehicle)
        if data is None:
            return

        t.transform = mono_transform_to_ros_transform(data)
        # t.transform.translation.x = 0
        # t.transform.translation.y = 0
        # t.transform.translation.z = 0
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.name
        marker = get_vehicle_marker(
            data, header=header, marker_id=0, is_player=True)
        self.process_msg_fun(self.name, marker)
        self.process_msg_fun('tf', t)


class NonPlayerAgentsHandler(AgentObjectHandler):
    """
    Convert non-player agents into monodrive_ros_bridge messages
    """

    def __init__(self, name, **kwargs):
        super(NonPlayerAgentsHandler, self).__init__(name, **kwargs)

    def process_msg(self, data, cur_time):
        """
        :param data: list of mono_server_pb2.Agent return by mono API,
        with field 'id', 'vehicle', 'pedestrian', 'traffic_light', 'speed_limit_sign'

        :return:
        """
        vehicles = [(self.get_marker_id(agent.id), agent.vehicle)
                    for agent in data if agent.HasField('vehicle')]

        header = Header(stamp=cur_time, frame_id=self.world_frame)
        if not (vehicles):
            return
        markers = [
            get_vehicle_marker(vehicle, header, agent_id)
            for agent_id, vehicle in vehicles
        ]
        marker_array = MarkerArray(markers)
        self.process_msg_fun('vehicles', marker_array)


def get_vehicle_marker(object, header, marker_id=0, is_player=False):
    """
    Return a marker msg

    :param object: mono agent object (pb2 object (vehicle, pedestrian or traffic light))
    :param header: monodrive_ros_bridge header (stamp/frame_id)
    :param marker_id: a marker id (int32)
    :param is_player: True if player else False (usefull to change marker color)
    :return: a monodrive_ros_bridge marker msg
    """
    marker = Marker(header=header)
    marker.action = Marker.ADD
    marker.color.a = 0.7
    if is_player:
        marker.color.g = 1
        marker.color.r = 0
        marker.color.b = 0
    else:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0

    marker.id = marker_id
    marker.text = "id = {}".format(marker_id)
    update_marker_pose(object, marker)

    return marker


def update_marker_pose(object, base_marker):
    """
    Update pose of a marker based on mono information

    :param object: pb2 mono object
    :param base_marker: marker to update pose
    """
    #print("ego m", object.matrix)

    ros_transform = mono_transform_to_ros_transform(
        # mono_Transform(object.bounding_box.transform) *
        mono_Transform(object))
    base_marker.pose = ros_transform_to_pose(ros_transform)


    base_marker.scale.x = 4.83
    base_marker.scale.y = 2.05
    base_marker.scale.z = 0.75 #1.25

    base_marker.pose.position.z += base_marker.scale.z / 2.0

    #print("base_marker t", base_marker.pose.position)
    #print("base_marker r", base_marker.pose.orientation)

    # base_marker.scale.x = object.bounding_box.extent.x * 2.0
    # base_marker.scale.y = object.bounding_box.extent.y * 2.0
    # base_marker.scale.z = object.bounding_box.extent.z * 2.0

    base_marker.type = Marker.CUBE
