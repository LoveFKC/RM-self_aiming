import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image
from auto_aim_interfaces.msg import TotalValue
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

from utils.FirstTracker import start_tracker
from utils.buffTracker import F_BuffTracker, BBox
from utils.parameterUtils import Parameter
from utils.angleProcessor import bigPredictor, smallPredictor, angleObserver, trans, clock, mode
from utils.camera_to_world import Message_to_send



class ImageProcessor_predict(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.subscription  # 防止未使用变量警告

        # 创建发布者，用于发布处理后的结果
        self.publisher = self.create_publisher(
            TotalValue,
            'total_value_topic',
            qos_profile_sensor_data
        )

        self.bridge = CvBridge()
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        yaml_path = os.path.join(current_dir, "parameter.yaml")
        self.param = Parameter(yaml_path)
        
        self.isImshow = True
        self.frameCount = 0
        self.angles = []
        self.xy = []
        self.color = 'blue'
        self.moveMode = mode.small
        self.freq = 50
        self.deltaT = 0.2
        self.interval = int(self.freq * self.deltaT)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('无法转换图像: {}'.format(str(e)))
            return

        self.frameCount += 1
        print(self.frameCount)

        if self.frameCount == 1:
            #rect = cv2.selectROI('roi', frame)
            #rect2 = cv2.selectROI("roi2", frame)
            rect,rect2 = start_tracker(frame, self.color)
            if rect is None or rect2 is None:
            	self.frameCount = 0
            	return
            R_Box = BBox(rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
            fanBladeBox = BBox(rect2[0], rect2[1], rect2[0] + rect2[2], rect2[1] + rect2[3])
            self.R_center = [int(rect[0] + rect[2]/2), int(rect[1] + rect[3]/2)]
            self.tracker = F_BuffTracker(fanBladeBox, R_Box, self.param, isImshow=self.isImshow)
            if self.color == "red":
                self.observer = angleObserver(clockMode=clock.clockwise)
            elif self.color == "blue":
                self.observer = angleObserver(clockMode=clock.anticlockwise)
            if self.moveMode == mode.small:
                self.predictor = smallPredictor(freq=self.freq, deltaT=self.deltaT)
            elif self.moveMode == mode.big:
                self.predictor = bigPredictor(freq=self.freq, deltaT=self.deltaT)


        flag = self.tracker.update(frame, True)
        if not flag:
            print(False)
            raise ValueError("画面中没有扇叶了")
        x, y = self.tracker.fanBladeBox.center_2f - self.tracker.R_Box.center_2f
        angle = self.observer.update(x, y, self.tracker.radius)
        self.angles.append(angle)
        flag, deltaAngle = self.predictor.update(angle)
        if flag:
            angle = trans(x, y) + deltaAngle
            x = np.cos(angle) * self.tracker.radius
            y = np.sin(angle) * self.tracker.radius
            x, y = np.array([x, y]) + self.tracker.R_Box.center_2f

            #发布数据
            total_value = Message_to_send(self.R_center,[x,y])
            self.publisher.publish(total_value)

            self.xy.append([x, y])

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor_predict()

    rclpy.spin(image_processor)

    # Shutdown
    image_processor.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    '''
        cv2.circle(frame, (int(x), int(y)), 10, (0, 255, 0), -1)
        cv2.putText(frame, "当前预测", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        if len(self.xy) >= self.interval:
            x, y = self.xy[len(self.xy) - self.interval]
            cv2.circle(frame, (int(x), int(y)), 10, (0, 255, 0), -1)
            cv2.putText(frame, "0.2秒前预测", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            pass

        cv2.rectangle(frame, self.tracker.fanBladeBox.p1, self.tracker.fanBladeBox.p2, (0, 255, 0), 3)
        if self.isImshow:
            cv2.imshow("frame", frame)
            c = cv2.waitKey(1)
            if c == ord('q') or c == ord('Q'):
                self.destroy_node()
    '''
