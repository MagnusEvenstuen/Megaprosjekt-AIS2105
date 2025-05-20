import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String
import cv2
from cv_bridge import CvBridge
import numpy as np

def find_colored_qubes(image, find_color, threshold):
    # Gets the dimensions of the image
    height, width, _ = image.shape
    image_center = [width / 2, height / 2]

    # Split into blue, green, red channels
    b, g, r = cv2.split(image)

    # Sum the channels that should not be found and calculate the difference
    if findColor == 3:  # YELLOW (use HSV)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        newImage = mask
    elif find_color == 2:      # Red
        sum_unused = cv2.add(b, g)
        difference = cv2.subtract(r, sum_unused)
    elif find_color == 1:    # Green
        sum_unused = cv2.add(b, r)
        difference = cv2.subtract(g, sum_unused)
    else:                    # Blue
        sum_unused = cv2.add(g, r)
        difference = cv2.subtract(b, sum_unused)

    # Apply threshold to get binary image
    _, new_image = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)

    # Detect the shapes
    contours, _ = cv2.findContours(new_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    square_contours = []
    for contour in contours:
        contour_length = cv2.arcLength(contour, True)
        approximation = cv2.approxPolyDP(contour, 0.04 * contour_length, True)
        if len(approximation) == 4 and cv2.isContourConvex(approximation):
            x, y, w, h = cv2.boundingRect(approximation)
            aspect_ratio = float(w) / h
            if 0.5 <= aspect_ratio <= 1.5 and cv2.contourArea(approximation) > 50:
                square_contours.append(approximation)

    result_image = np.zeros_like(new_image)
    square_center_image = []

    if square_contours:
        pixels_to_keep = np.zeros_like(new_image)
        cv2.drawContours(pixels_to_keep, square_contours, -1, 255, -1)
        result_image = cv2.bitwise_and(image, image, mask=pixels_to_keep)

        for square in square_contours:
            # Compute average corner coordinates
            coords = square[:, 0, :]
            avg_x = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) / 4
            avg_y = (coords[0][1] + coords[1][1] + coords[2][1] + coords[3][1]) / 4
            rounded_center = [round(avg_x), round(avg_y)]
            square_center_image = [
                round(rounded_center[0] - image_center[0]),
                round(image_center[1] - rounded_center[1])
            ]

    return square_center_image


class distance_publisher_image_subscriber(Node):

    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'distance_to_qube_center',
            10
        )
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Process image to find squares
        square_centers = find_colored_qubes(cv_image, find_color=2, threshold=10)
        
        if square_centers:
            publish_msg = Float64MultiArray()
            publish_msg.data = square_centers
            self.publisher.publish(publish_msg)
            self.get_logger().info(f'Published coordinates: {square_centers}')


def main(args=None):
    rclpy.init(args=args)

    distance_to_square_center_publisher = distance_publisher_image_subscriber()

    rclpy.spin(distance_to_square_center_publisher)

    distance_to_square_center_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
