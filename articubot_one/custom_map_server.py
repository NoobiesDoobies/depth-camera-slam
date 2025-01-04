import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import numpy as np
from PIL import Image
from ament_index_python.packages import get_package_share_directory
import os

class CustomMapServer(Node):
    def __init__(self):
        super().__init__('custom_map_server')

        # Publisher for the /map topic
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Load the map
        package_path = get_package_share_directory('articubot_one')
        yaml_path = os.path.join(package_path, 'maps', 'lidar_map2.yaml')
        self.map_msg = self.load_map(yaml_path)

        # Create a timer to publish the map every 2 seconds
        self.timer = self.create_timer(0.1, self.publish_map)

        self.get_logger().info('Custom Map Server is running...')

    def load_map(self, yaml_filename):
        # Load the YAML file
        with open(yaml_filename, 'r') as file:
            map_data = yaml.safe_load(file)

        # Get map metadata
        package_path = get_package_share_directory('articubot_one')
        image_path = os.path.join(package_path, 'maps', map_data['image'])
        resolution = map_data['resolution']
        origin = map_data['origin']
        negate = map_data['negate']
        occupied_thresh = map_data['occupied_thresh']
        free_thresh = map_data['free_thresh']

        # Load the image
        image = Image.open(image_path)
        image = image.convert('L')  # Convert to grayscale
        np_image = np.array(image)

        # Convert image to occupancy grid
        occupancy_grid = np.where(np_image > 255 * free_thresh, 0, 100)
        occupancy_grid = occupancy_grid.flatten().tolist()

        # Create an OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = resolution
        map_msg.info.width = image.width
        map_msg.info.height = image.height
        map_msg.info.origin.position.x = float(origin[0])
        map_msg.info.origin.position.y = float(origin[1])
        map_msg.info.origin.position.z = float(origin[2])
        map_msg.info.origin.orientation.w = 1.0

        # Fill in the data
        map_msg.data = occupancy_grid

        return map_msg

    def publish_map(self):
        # Update the timestamp and publish the map
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)
        self.get_logger().info('Map published!')


def main(args=None):
    rclpy.init(args=args)
    custom_map_server = CustomMapServer()
    rclpy.spin(custom_map_server)
    custom_map_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
