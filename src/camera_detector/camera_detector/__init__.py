import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String
import cv2
from cv_bridge import CvBridge
import numpy as np

def findColoredQubes(image, findColor, threshold):
    #Gets the dimentions of the image
    height, width, _ = image.shape
    imageCenter = [width/2, height/2]

    #Split into blue, green, red channels
    b, g, r = cv2.split(image)

    #Sum the channels that shoul not be found and claculate the difference between found value and wanted color
    if findColor == 2:      #Red
        sumUnused = cv2.add(b, g)
        difference = cv2.subtract(r, sumUnused)
    elif findColor == 1:    #Green
        sumUnused = cv2.add(b, r)
        difference = cv2.subtract(g, sumUnused)
    else:                   #Blue
        sumUnused = cv2.add(g, r)
        difference = cv2.subtract(b, sumUnused)


    #Apply threshold to get binary image
    _, newImage = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)

    #Detect the shapes RETR_EXTERNAL looks at external shapes, CHAIN_APPROX_SIMPLE compremices points at edge
    contours, _ = cv2.findContours(newImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    squareContours = []
    for contour in contours:
        #arcLength looks at shapes circumfrence, sees if it is a loop
        contourLength = cv2.arcLength(contour, True)
        #Approximates the conture
        approximation = cv2.approxPolyDP(contour, 0.04 * contourLength, True)
        #Checks if the number of corners is 4
        if len(approximation) == 4:
            #Checks that there are no curvs
            if cv2.isContourConvex(approximation):
                #Get bounding rectangle aspect ratio
                x, y, w, h = cv2.boundingRect(approximation)
                aspect_ratio = float(w)/h

                #Allow aspect ratio between 0.8 and 1.2, meaning almost squares
                if 0.5 <= aspect_ratio <= 1.5:
                    #Checks if the area is big enough
                    if cv2.contourArea(approximation) > 50:
                        squareContours.append(approximation)

    resultImage = np.zeros_like(newImage)

    squareCenterImage = []
    #Creates new image if it finds squares
    if squareContours:
        #Creates new image with same size as old image
        pixelsToKeep = np.zeros_like(newImage)
        #Sets everything black exept the squares
        cv2.drawContours(pixelsToKeep, squareContours, -1, 255, -1)

        resultImage = cv2.bitwise_and(image, image, mask=pixelsToKeep)
        
        #Prints the senter of the squares compared to the center of the image
        for square in squareContours:
            squareCenter = [round((square[0][0][0] + square[1][0][0] + square[2][0][0] + square[3][0][0])/4),
                            round((square[0][0][1] + square[1][0][1] + square[2][0][1] + square[3][0][1])/4)]
            squareCenterImage = [squareCenter[0] - imageCenter[0],
                                 imageCenter[1] - squareCenter[1]]
    
    return squareCenterImage


class DistancePublisherImageSubscriber(Node):

    def __init__(self):
        super().__init__('distancePublisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'distanceToQubeCenter', 10)
        #Add camera subscription and bridge
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        #Convert ROS Image to OpenCV format
        cvImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process image to find squares
        squareCenters = findColoredQubes(cvImage, findColor=2, threshold=10)
        
        if squareCenters:
            msg = Float64MultiArray()
            msg.data = squareCenters
            self.publisher.publish(msg)
            self.get_logger().info('Published coordinates: %s' % str(squareCenters))

def main(args=None):
    rclpy.init(args=args)

    distanceToSquareCenterPublisher = DistancePublisherImageSubscriber()

    rclpy.spin(distanceToSquareCenterPublisher)

    distanceToSquareCenterPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()