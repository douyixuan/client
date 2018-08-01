
from geometry_msgs.msg import Transform, TransformStamped
import tf


class WorldMapHandler(object):

    def __init__(self, world_frame='monodrive', process_msg_fun=None):
        self.world_frame = world_frame
        self.process_msg_fun = process_msg_fun

    def process_msg(self, cur_time):
        t = TransformStamped()
        t.header.stamp = cur_time
        #t.header.frame_id = self.world_frame
        t.child_frame_id = self.world_frame
        t.transform = Transform()

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # self.process_msg_fun('tf', t)