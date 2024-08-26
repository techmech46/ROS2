import rclpy  # ROS 2 Python client library
import cv2  # OpenCV library for image processing
from rclpy.node import Node  # Base class for a ROS 2 node
from cv_bridge import CvBridge  # Converts between ROS 2 Image messages and OpenCV images
from sensor_msgs.msg import Image  # ROS 2 message type for image data

class Capture(Node):
    def __init__(self):
        super().__init__('video_subscriber')  # Initialize the node with the name 'video_subscriber'

        # Create a subscription to the '/camera1/image_raw' topic with the Image message type
        # The process_data method will be called every time a new message is received
        self.subscriber = self.create_subscription(Image, '/camera1/image_raw', self.process_data, 10)

        # Initialize video writer to save the output video to a file
        self.out = cv2.VideoWriter(
            '/home/kashyap/output.avi',  # Path to save the video file
            cv2.VideoWriter_fourcc('M','J','P','G'),  # Codec for the video (MJPG)
            10,  # Frames per second (FPS)
            (512, 512)  # Frame size (width, height)
        )

        # Initialize CvBridge to convert between ROS 2 Image messages and OpenCV images
        self.bridge = CvBridge()

    def process_data(self, data):
        # Convert the ROS 2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(data)

        # Write the frame to the video file
        self.out.write(frame)

        # Save a single frame as an image file (snapshot)
        self.img = cv2.imwrite('/home/kashyap/shot.png', frame)

        # Display the frame in a window named "output"
        cv2.imshow("output", frame)

        # Wait for a key press and then close the window
        cv2.waitKey()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = Capture()  # Create an instance of the Capture node
    rclpy.spin(node)  # Keep the node running to process incoming messages
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Entry point for the script
