"""
take_photo.py

ROS2 node that listens to the robot's gripper camera.
Press Enter to save a photo, type 'q' to stop.
Photos are saved to ~/photos/ as photo_0000.jpg, photo_0001.jpg, ...
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# Directory on the laptop where photos will be saved.
SAVE_DIR = os.path.expanduser('~/photos')

# ROS topic on which the robot's gripper camera publishes its frames.
TOPIC = '/gripper_camera/image_raw'


class TakePhoto(Node):
    """ROS2 node that receives camera frames and saves them as JPEGs on demand."""

    def __init__(self):
        super().__init__('take_photo')

        # Create the photo directory if it does not exist yet.
        os.makedirs(SAVE_DIR, exist_ok=True)

        # CvBridge translates ROS Image messages into OpenCV images.
        self.bridge = CvBridge()

        # Most recent camera frame; None until the first frame arrives.
        self.latest_frame = None

        # Counter used to build the filename (photo_0000.jpg, photo_0001.jpg, ...).
        self.photo_count = 0

        # Subscribe to the camera topic. Queue size 10 means ROS keeps at most
        # 10 unprocessed messages before dropping older ones.
        self.create_subscription(Image, TOPIC, self._image_callback, 10)
        self.get_logger().info(f'Listening on {TOPIC} — press Enter to take a photo, type "q" to stop.')

    def _image_callback(self, msg):
        """Called for every new camera frame that arrives on the topic."""
        # Convert the ROS Image message to a BGR OpenCV matrix.
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def save_photo(self):
        """Save the most recent frame as a JPEG file."""
        if self.latest_frame is None:
            self.get_logger().warn('No camera frame received yet.')
            return

        filename = os.path.join(SAVE_DIR, f'photo_{self.photo_count:04d}.jpg')
        cv2.imwrite(filename, self.latest_frame)
        self.photo_count += 1
        self.get_logger().info(f'Photo saved: {filename}')


def main(args=None):
    """
    Entry point of the node.

    ROS spin runs in a separate thread so that incoming messages are processed
    continuously while the main thread waits for keyboard input from the user.
    """
    rclpy.init(args=args)
    node = TakePhoto()

    # Run rclpy.spin in the background (daemon=True so the thread stops
    # automatically when the main program exits).
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            cmd = input()
            if cmd.strip().lower() == 'q':
                break
            node.save_photo()
    except (KeyboardInterrupt, EOFError):
        # Ctrl+C or a closed stdin shuts down the program cleanly.
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
