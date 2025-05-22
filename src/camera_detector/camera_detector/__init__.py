import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String
import cv2
from cv_bridge import CvBridge
import numpy as np

def find_colored_qubes(image, find_color):
    # Gets the dimensions of the image
    height, width, _ = image.shape
    image_center = [width / 2, height / 2]

    # Sum the channels that should not be found and calculate the difference
    if find_color == 1:  #Yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_limit = np.array([20, 60, 20])
        upper_limit = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_limit, upper_limit)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        new_image = mask
    elif find_color == 0:      #Red
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_limit = np.array([160, 100, 20])
        upper_limit = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_limit, upper_limit)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        new_image = mask
    elif find_color == 3:    #Green
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_limit = np.array([35, 60, 20])
        upper_limit = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_limit, upper_limit)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        new_image = mask
    else:                   #Blue
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_limit = np.array([90, 100, 20])
        upper_limit = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_limit, upper_limit)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        new_image = mask

    # Detect the shapes
    contours, _ = cv2.findContours(new_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    square_contours = []
    for contour in contours:
        contour_length = cv2.arcLength(contour, True)
        approximation = cv2.approxPolyDP(contour, 0.04 * contour_length, True)
        if len(approximation) == 4 and cv2.isContourConvex(approximation):
            x, y, w,  h = cv2.boundingRect(approximation)
            aspect_ratio = float(w) / h
            if 0.5 <= aspect_ratio <= 1.5 and cv2.contourArea(approximation) > 50:
                square_contours.append(approximation)

    square_center_image = []

    if square_contours:
        pixels_to_keep = np.zeros_like(new_image)
        cv2.drawContours(pixels_to_keep, square_contours, -1, 255, -1)

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
        self.has_published = False

    def image_callback(self, msg):
        if self.has_published:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        square_centers = [-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000]

        red_centers = find_colored_qubes(cv_image, find_color=0)
        if red_centers:
            square_centers[0:2] = red_centers

        yellow_centers = find_colored_qubes(cv_image, find_color=1)
        if yellow_centers:
            square_centers[2:4] = yellow_centers
            
        blue_centers = find_colored_qubes(cv_image, find_color=2)
        if blue_centers:
            square_centers[4:6] = blue_centers

        green_centers = find_colored_qubes(cv_image, find_color=3)
        if green_centers:
            square_centers[6:8] = green_centers
        
        publish_msg = Float64MultiArray()
        publish_msg.data = square_centers
        self.publisher.publish(publish_msg)
        self.has_published = True
        self.get_logger().info(f'Published coordinates: {square_centers}')


def main(args=None):
    rclpy.init(args=args)

    distance_to_square_center_publisher = distance_publisher_image_subscriber()

    rclpy.spin(distance_to_square_center_publisher)

    distance_to_square_center_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
