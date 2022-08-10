
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf
import TwoDToThreeD


class Image_sender:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
        self.br = tf.TransformBroadcaster()
        self.trans = TwoDToThreeD()
        self.mouse_point = "mouse_point"

    def on_EVENT_LBUTTONDOWN(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            t_ = self.trans.convert_(x, y)
            x3d = t_[0]
            y3d = t_[1]
            z3d = t_[2]
            xy = "[%d,%d],[%.2f,%.2f,%.2f]" % (x, y, x3d, y3d, z3d)
            print(xy)
            cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
            cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                        1.0, (0, 0, 0), thickness=1)
            print(x, y)

            try:
                self.br.sendTransform((x3d, y3d, z3d),
                                      tf.transformations.quaternion_from_euler(0, 0, 0),
                                      rospy.Time.now(),
                                      self.mouse_point,
                                      "world")
            except CvBridgeError as e:
                print(e)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.on_EVENT_LBUTTONDOWN)
        cv2.imshow("image", cv_image)





def main(args):
    ic = Image_sender()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

