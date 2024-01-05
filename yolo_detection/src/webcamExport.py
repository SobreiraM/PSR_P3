#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    # initial setup
    capture = cv2.VideoCapture(0)
    window_name = 'Webcam'
    cv2.namedWindow(window_name,cv2.WINDOW_AUTOSIZE)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    while True:

        bridge = CvBridge()

        pub = rospy.Publisher('Image', Image, queue_size=1)
        rospy.init_node('publisher', anonymous=True)

        while not rospy.is_shutdown():
            _, image = capture.read()  # get an image from the camera
        
            
            ros_image = bridge.cv2_to_imgmsg(image, encoding='bgr8')

            image_to_send = ros_image

            cv2.imshow(window_name, image)
            if cv2.waitKey(25) == ord(' '):
              break


            pub.publish(image_to_send)
            rospy.sleep(0.05)
        
        capture.release()
        cv2.destroyAllWindows()
        break
        


if __name__ == '__main__':
    main()