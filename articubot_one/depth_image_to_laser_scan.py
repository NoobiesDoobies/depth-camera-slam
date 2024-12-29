import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from cv_bridge import CvBridge
import cv2

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/articubot_one/front/depth/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # self.pcl_subscription = self.create_subscription(
        #     PointCloud2,
        #     '/omni_bot/front/points',
        #     self.pcl_listener_callback,
        #     10)
        # self.pcl_subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.bridge = CvBridge()

    def pcl_listener_callback(self, msg):
        self.get_logger().info('Received point cloud')
        try:
            self.get_logger().info(f'Fields: {msg.fields}')
            self.get_logger().info(f'Height: {msg.height}')
            self.get_logger().info(f'Width: {msg.width}')
            self.get_logger().info(f'Point Step: {msg.point_step}')
            self.get_logger().info(f'Row Step: {msg.row_step}')
            self.get_logger().info(f'Is Dense: {msg.is_dense}')

            #Fields: [
            #  sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
            #  sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
            #  sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1),
            #  sensor_msgs.msg.PointField(name='rgb', offset=16, datatype=7, count=1)]

            # Get first and last y values
            delta_y = msg.data[4] - msg.data[msg.row_step * (msg.height - 1) + 4]
            self.get_logger().info(f'Delta y: {delta_y}')

            pass

        except Exception as e:
            self.get_logger().error(f'Error converting image')

    def listener_callback(self, msg):
        self.get_logger().info('Received depth image')
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Display the image
            cv2.imshow("Depth Image", cv_image)

            # Get total rows and columns of the image
            rows, cols = cv_image.shape

            # Create a LaserScan message
            laser_scan_msg = LaserScan()
            msg.header.frame_id = 'front_cam_link'
            laser_scan_msg.header = msg.header

            fov = 1.51844
            laser_scan_msg.angle_min = -fov/2
            laser_scan_msg.angle_max = fov/2
            laser_scan_msg.angle_increment = fov / cols
            laser_scan_msg.time_increment = 0.0
            laser_scan_msg.scan_time = 0.0
            laser_scan_msg.range_min = 0.0
            laser_scan_msg.range_max = 20.0

            self.get_logger().info(f'Rows: {rows}, Cols: {cols}')
            # Create the ranges array
            ranges = cv_image[rows//2 + 20, :]
            # convert ranges to float32[]
            ranges = ranges.astype('float32')
            # self.get_logger().info(f'Ranges: {ranges}')
            ranges = ranges.tolist()

            # Assign the ranges to the LaserScan message
            laser_scan_msg.ranges = ranges

            # Publish the LaserScan message
            self.publisher.publish(laser_scan_msg)
           
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_image_subscriber)
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()