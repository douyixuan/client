import matplotlib.pyplot as plt
import math
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import rospy
import tf


def haversine_distance(origin, destination):
    lat1, lon1 = origin
    lat2, lon2 = destination
    radius = 6371000  # meters

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) * math.sin(dlon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return radius * c


def cm2inch(value):
    return value/2.54


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

        w = 10000
        for road_index in range(0, self.map_data.road_count()):
            road = self.map_data.road(road_index)
            for lane_index in range(0, road.lane_count()):
                lane = road.lane(lane_index)
                lane_points = lane.points
                if lane.width > 0:
                    w = min(w, lane.width)

                x = list(map(lambda p: p['x'] / 100, lane_points))
                y = list(map(lambda p: -p['y'] / 100, lane_points))

                xy_points = np.column_stack((x, y))
                points_by_lane.append(xy_points)

        x_combined = []
        y_combined = []
        for points in points_by_lane:
            x_combined = np.append(x_combined, points[:, 0])
            y_combined = np.append(y_combined, points[:, 1])

        bounds = self.map_data.bounds
        w = haversine_distance((bounds["min"]["x"], bounds["min"]["y"]), (bounds["max"]["x"], bounds["min"]["y"]))
        h = haversine_distance((bounds["min"]["x"], bounds["min"]["y"]), (bounds["min"]["x"], bounds["max"]["y"]))
        plt.rcParams["figure.figsize"] = [cm2inch(w/10), cm2inch(h/10)]
        plt.rcParams["axes.xmargin"] = plt.rcParams["axes.ymargin"] = 0.0
        plt.rcParams["figure.frameon"] = False
        plt.rcParams["figure.subplot.bottom"] = 0.
        plt.rcParams['figure.subplot.hspace'] = 0.
        plt.rcParams['figure.subplot.left'] = 0.
#        plt.rcParams['figure.subplot.right'] = 0.
#        plt.rcParams['figure.subplot.top'] = 0.
        plt.rcParams['figure.subplot.wspace'] = 0.
        plt.rcParams['savefig.pad_inches'] = 0.
        plt.rcParams['legend.frameon'] = False

        plt.axis('off')

        rospy.loginfo("lane width: {0}".format(w))
        plt.plot(x_combined, y_combined, 'g.-', linestyle='None', linewidth=w/100)

        plt.savefig("map.png", bbox_inches='tight', transparent=True, pad_inches=0)

        map_image = Image.open("map.png")
        map_image.load()

        self.map_image = np.asarray(map_image, dtype="int32")
        self.build_map_message()

    def build_map_message(self):
        self.map_msg = map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'

        # form array for map
        map_img = self.map_image
        # extract green channel, invert, scale to range 0..100, convert to int8
        map_img = (100 - map_img[..., 1] * 100.0 / 255).astype(np.int8)
        map_msg.data = map_img.ravel().tolist()

        # set up general info
        bounds = self.map_data.bounds
        horiz = haversine_distance((bounds["min"]["x"], bounds["min"]["y"]), (bounds["max"]["x"], bounds["min"]["y"]))
        vert = haversine_distance((bounds["min"]["x"], bounds["min"]["y"]), (bounds["min"]["x"], bounds["max"]["y"]))
        rospy.loginfo("{0},{1} ({2},{3})".format(horiz, vert, map_img.shape[1], map_img.shape[0]))
        resolution = map_img.shape[1] / horiz / 10
        map_msg.info.resolution = resolution
        map_msg.info.width = map_img.shape[1]
        map_msg.info.height = map_img.shape[0]

        # set up origin orientation
        rot = self.map_data.default_start["rotation"]
        rospy.loginfo(rot)
        quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.radians(rot["yaw"]))
        map_msg.info.origin.orientation.x = quat[0]
        map_msg.info.origin.orientation.y = quat[1]
        map_msg.info.origin.orientation.z = quat[2]
        map_msg.info.origin.orientation.w = quat[3]

        # set up origin position
#        start = self.map_data.default_start
#        rospy.loginfo(start)
#        x = haversine_distance((bounds["max"]["x"], bounds["max"]["y"]), (start["location"]["x"], bounds["max"]["y"]))
#        y = haversine_distance((bounds["max"]["x"], bounds["max"]["y"]), (bounds["max"]["x"], start["location"]["y"]))
        map_msg.info.origin.position.x = -vert
        map_msg.info.origin.position.y = -horiz / 2
        map_msg.info.origin.position.z = 0
        rospy.loginfo("{0} -> {1}".format(bounds, map_msg.info.origin.position))

    def send_map(self):
        self.map_pub.publish(self.map_msg)