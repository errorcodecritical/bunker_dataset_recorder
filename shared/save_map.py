import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map_local',
            self.save_map,
            10
        )

    def save_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Convert ROS values:
        # -1 unknown, 0 free, 100 occupied
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        img[data == -1] = 205

        Image.fromarray(img).save("kalhan_occupation_map.pgm")
        self.get_logger().info("Map saved!")

        rclpy.shutdown()

rclpy.init()
node = MapSaver()
rclpy.spin(node)
