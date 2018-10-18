import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import rospy
import tf


class LaneData(object):
    def __init__(self, data):
        self.__dict__ = data


class RoadData(object):
    def __init__(self, data):
        self.__dict__ = data

    def lane_count(self):
        return len(self.lanes)

    def lane(self, lane_index):
        return LaneData(self.lanes[lane_index])


class MapData(object):
    def __init__(self, data):
        self.__dict__ = data

    def road_count(self):
        return len(self.roads)

    def road(self, road_index):
        return RoadData(self.roads[road_index])


class Map(object):

    def __init__(self, map_data, topic='/map'):
        self.map_data = MapData(map_data)

        self.map_pub = rospy.Publisher(
            topic, OccupancyGrid, queue_size=10, latch=True)

        self.update_map()

    def update_map(self):
        points_by_lane = []

        for road_index in range(0, self.map_data.road_count()):
            road = self.map_data.road(road_index)
            for lane_index in range(0, road.lane_count()):
                lane = road.lane(lane_index)
                lane_points = lane.points

                x = list(map(lambda p: p['x'] / 100, lane_points))
                y = list(map(lambda p: -p['y'] / 100, lane_points))

                xy_points = np.column_stack((x, y))
                points_by_lane.append(xy_points)

        x_combined = []
        y_combined = []
        for points in points_by_lane:
            x_combined = np.append(x_combined, points[:, 0])
            y_combined = np.append(y_combined, points[:, 1])

        handle = plt.plot(x_combined, y_combined, 'g.-', linestyle='None')[0]
        handle.set_xdata(x_combined)
        handle.set_ydata(y_combined)
        plt.axis('off')
        plt.savefig("map.png")

        map_image = Image.open("map.png")
        map_image.load()

        self.map_image = np.asarray(map_image, dtype="int32")
        self.build_map_message()

    def build_map_message(self):
        self.map_msg = map_msg = OccupancyGrid()

        # form array for map
        map_img = self.map_image
        # extract green channel, invert, scale to range 0..100, convert to int8
        map_img = (100 - map_img[..., 1] * 100.0 / 255).astype(np.int8)
        map_msg.data = map_img.ravel().tolist()

        # set up general info
        map_msg.info.resolution = 100
        map_msg.info.width = map_img.shape[1]
        map_msg.info.height = map_img.shape[0]

        # set up origin orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, np.pi)
        map_msg.info.origin.orientation.x = quat[0]
        map_msg.info.origin.orientation.y = quat[1]
        map_msg.info.origin.orientation.z = quat[2]
        map_msg.info.origin.orientation.w = quat[3]

        # set up origin position
        position = self.map_data.bounds
        map_msg.info.origin.position.x = position['min']['x']
        map_msg.info.origin.position.y = position['min']['y']
        map_msg.info.origin.position.z = position['min']['z']

    def send_map(self):
        self.map_pub.publish(self.map_msg)