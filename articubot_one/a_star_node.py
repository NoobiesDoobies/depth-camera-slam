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
import random
import heapq

class AStarPathfinder(Node):
    def __init__(self):
        super().__init__('astar_pathfinder')

        # Subscribe to the map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

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
        self.robot_position = (0,0)  # Store the robot position
        self.robot_orientation = None  # Store the robot orientation (angle)

        self.goal_position = (0,0)  # Store the goal position
        self.goal_orientation = None  # Store the goal orientation (angle)

        self.path_points = []  # To store the path

        self.bridge = CvBridge()  # Initialize cv_bridge

    def goal_pose_callback(self, msg):
        """ Callback for receiving the goal pose """
        self.goal_position = (msg.pose.position.x + self.robot_position[0], -msg.pose.position.y + self.robot_position[1])
        self.goal_orientation = self.quaternion_to_angle(msg.pose.orientation)
        
        start_x, start_y = self.convert_real_to_image_coordinate(self.robot_position[0], self.robot_position[1])
        start = (start_x, start_y)


        goal_x, goal_y = self.convert_real_to_image_coordinate(self.goal_position[0], self.goal_position[1])

        goal = (goal_x, goal_y)

        self.path_points = self.a_star_search(start, goal, self.grid_map)

        print(start_x, start_y)
        print(goal_x, goal_y)

    def map_callback(self, msg):
        """ Callback for receiving the map """
        self.grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        # Get the transform from odom to base_link
        self.get_robot_transform()

        # Run A* Algorithm
        start_x, start_y = self.convert_real_to_image_coordinate(self.robot_position[0], self.robot_position[1])
        start = (start_x, start_y)


        goal_x, goal_y = self.convert_real_to_image_coordinate(self.goal_position[0], self.goal_position[1])
        goal_x += start_x
        goal_y += start_y
        goal = (goal_x, goal_y)


        # start = (self.robot_position[0], self.robot_position[1])
        # goal = (self.goal_position[0], self.goal_position[1])

        # self.path_points = self.a_star_search(start, goal, self.grid_map)

        # Publish the map image with the robot's position, orientation, and path
        self.publish_map_image()

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
    
    def convert_real_to_image_coordinate(self, x, y):
        x_image = int((x - self.origin.x) / self.resolution)
        y_image = int((y + self.origin.y + self.map_height*self.resolution) / self.resolution)

        return x_image, y_image
    
    def convert_image_to_real_coordinate(self, x, y):
        x_real = int((x * self.resolution) + self.origin.x)
        y_real = int((y * self.resolution) - self.origin.y - self.map_height*self.resolution)

        return x_real, y_real

    def publish_map_image(self):
        """ Convert the occupancy grid map to an image and publish it with the robot's position, orientation, and path """
        # Convert the occupancy grid to a color image for visualization
        image_data = np.uint8(self.grid_map * 255)  # Convert grid to 0-255 values
        image_data = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)  # Convert to BGR (3 channels)

        if self.robot_position and self.robot_orientation is not None:
            # Convert the robot position from meters to grid coordinates (pixels)
            robot_x, robot_y = self.convert_real_to_image_coordinate(self.robot_position[0], self.robot_position[1])
            
            # Draw a red arrow to indicate the robot's orientation
            arrow_length = 10  # Arrow length in pixels
            angle = -self.robot_orientation  # Robot's heading in radians
            end_x = int(robot_x + arrow_length * np.cos(angle))
            end_y = int(robot_y + arrow_length * np.sin(angle))
            # cv2.arrowedLine(image_data, (robot_x, robot_y), (end_x, end_y), (0, 0, 255), 1, tipLength=0.05)
            cv2.circle(image_data, (robot_x, robot_y), 1, (0, 0, 255), -1)

            goal_x, goal_y = self.convert_real_to_image_coordinate(self.goal_position[0], self.goal_position[1])
            # goal_x = int((self.goal_position[0] - self.origin.x) / self.resolution)
            # goal_y = int((-self.goal_position[1] + self.origin.y + self.map_height*self.resolution) / self.resolution)

            cv2.circle(image_data, (goal_x, goal_y), 1, (0, 255, 0), -1)

        # Draw the path
        if self.path_points:
            for i in range(len(self.path_points) - 1):
                x1, y1 = self.path_points[i][0], self.path_points[i][1]
                x2, y2 = self.path_points[i + 1][0], self.path_points[i + 1][1]
                cv2.line(image_data, (x1, y1), (x2, y2), (255, 0, 0), 1)

        # Convert the NumPy array to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image_data, encoding="bgr8")

        # Publish the image
        self.image_pub.publish(ros_image)

    def heuristic(self, a, b):
        """ Heuristic function: Manhattan distance """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, node):
        """Get neighbors of a node in the grid_map."""
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])

            if 0 <= neighbor[0] < self.grid_map.shape[0] and 0 <= neighbor[1] < self.grid_map.shape[1]:
                if self.grid_map[neighbor[0]][neighbor[1]] == 0:  # Check if the cell is free
                    neighbors.append(neighbor)

        return neighbors
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def a_star_search(self, start, goal, grid_map):
        """A* search algorithm."""
        open_set = set()
        open_set.add(start)
        came_from = {}

        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

            if current == goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        return []
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
