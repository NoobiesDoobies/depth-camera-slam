import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import matplotlib.pyplot as plt

class AStarPathfinder(Node):
    def __init__(self):
        super().__init__('astar_pathfinder')

        # Subscribe to the map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publisher to publish the calculated path
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.grid_map = None
        self.map_width = None
        self.map_height = None
        self.resolution = None
        self.origin = None

        self.get_logger().info('A* Pathfinder Node Started')

    def map_callback(self, msg):
        """ Callback for receiving the map """
        self.grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        self.get_logger().info('Map received! Starting pathfinding...')
        
        # Display map directly in the callback (in the main thread)
        self.display_map()

        # Example start and goal positions (you can modify this)
        start = (10, 10)  # Grid coordinates
        goal = (40, 50)  # Grid coordinates

        # Run A* Algorithm
        path = self.a_star_search(start, goal)

        if path:
            self.publish_path(path)
        else:
            self.get_logger().info('No path found!')

    def display_map(self):
        """ Display the map using Matplotlib in the main thread """
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid_map, cmap='gray', origin='lower')
        plt.title('Subscribed Occupancy Grid Map')
        plt.colorbar(label='Occupancy Value')
        plt.xlabel('X (grid cells)')
        plt.ylabel('Y (grid cells)')
        plt.show()  # This will now be in the main thread

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
        self.get_logger().info('Path published!')


def main(args=None):
    rclpy.init(args=args)
    astar_pathfinder = AStarPathfinder()
    rclpy.spin(astar_pathfinder)
    astar_pathfinder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
