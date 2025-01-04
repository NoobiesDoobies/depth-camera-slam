import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import heapq
import cv2
from cv_bridge import CvBridge
from math import atan2
from tf2_ros import TransformListener, Buffer


class AStarPathfinder(Node):
    def __init__(self):
        super().__init__('astar_pathfinder')

        # Subscribe to the map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Set up tf2 subscription to get the transformation between odom and base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to publish the calculated path
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # Publisher to publish the image
        self.image_pub = self.create_publisher(Image, '/map_image', 10)

        self.grid_map = None
        self.map_width = None
        self.map_height = None
        self.resolution = None
        self.origin = None
        self.robot_position = None  # Store the robot position
        self.robot_orientation = None  # Store the robot orientation (angle)

        self.bridge = CvBridge()  # Initialize cv_bridge


    def map_callback(self, msg):
        """ Callback for receiving the map """
        self.grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        
        # Get the transform from odom to base_link
        self.get_robot_transform()

        # Publish the map image with the robot's position, orientation
        self.publish_map_image()

        # Example start and goal positions (you can modify this)
        start = (10, 10)  # Grid coordinates
        goal = (40, 50)  # Grid coordinates

        # Run A* Algorithm
        path = self.a_star_search(start, goal)

        if path:
            self.publish_path(path)
        else:
            self.get_logger().info('No path found!')

    def get_robot_transform(self):
        """ Get the transform between odom and base_link """
        try:
            # Get the transform from odom to base_link
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            # Extract position and orientation
            self.robot_position = (transform.transform.translation.x, -transform.transform.translation.y)
            self.robot_orientation = self.quaternion_to_angle(transform.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    def quaternion_to_angle(self, quaternion):
        """ Convert quaternion to yaw (angle in radians) """
        # Convert quaternion to angle (yaw) using atan2 of the quaternion's z and w components
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return atan2(siny_cosp, cosy_cosp)

    def publish_map_image(self):
        """ Convert the occupancy grid map to an image and publish it with the robot's position and orientation """
        # Convert the occupancy grid to a color image for visualization
        image_data = np.uint8(self.grid_map * 255)  # Convert grid to 0-255 values
        image_data = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)  # Convert to BGR (3 channels)

        if self.robot_position and self.robot_orientation is not None:
            # Convert the robot position from meters to grid coordinates (pixels)
            robot_x = int((self.robot_position[0] - self.origin.x) / self.resolution)
            # robot_y = int((self.map_height + (self.robot_position[1] - self.origin.y)) / self.resolution)
            robot_y = int((self.robot_position[1] + (self.origin.y) + self.map_height*self.resolution) / self.resolution)

            print(self.map_width, self.map_height)
            
            # Draw a red arrow to indicate the robot's orientation
            arrow_length = 10  # Arrow length in pixels
            angle = -self.robot_orientation  # Robot's heading in radians
            end_x = int(robot_x + arrow_length * np.cos(angle))
            end_y = int(robot_y + arrow_length * np.sin(angle))
            cv2.arrowedLine(image_data, (robot_x, robot_y), (end_x, end_y), (0, 0, 255), 1, tipLength=0.05)

        # Convert the NumPy array to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image_data, encoding="bgr8")

        # Publish the image
        self.image_pub.publish(ros_image)

    def a_star_search(self, start, goal):
        """ A* Algorithm Implementation """
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                if 0 <= neighbor[0] < self.grid_map.shape[0]:
                    if 0 <= neighbor[1] < self.grid_map.shape[1]:
                        if self.grid_map[neighbor[0]][neighbor[1]] == 0:
                            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                                continue

                            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                                came_from[neighbor] = current
                                gscore[neighbor] = tentative_g_score
                                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                                heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return False

    def publish_path(self, path):
        """ Publish the calculated path as a Path message """
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        for grid_point in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = grid_point[1] * self.resolution + self.origin.x
            pose.pose.position.y = grid_point[0] * self.resolution + self.origin.y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    astar_pathfinder = AStarPathfinder()
    rclpy.spin(astar_pathfinder)
    astar_pathfinder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
