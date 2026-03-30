import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class LineFollowing(Node):
    def __init__(self):
        super().__init__('LineFollowing')
        
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_lane_width = 300.0
        self.last_target_x = 160.0
        self.declare_parameter('roundabout_dir', 'right')

    def get_line_centers(self, mask, y_steps):
        centers = {}
        for y in y_steps:
            row = mask[y, :]
            indices = np.where(row > 0)[0]
            if len(indices) > 0:
                centers[y] = int(np.mean(indices))
        return centers

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        height, width, _ = cv_image.shape
        roi_top = int(height * 0.5)
        roi_image = cv_image[roi_top:height, 0:width]
        hsv_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.bitwise_or(cv2.inRange(hsv_image, lower_red1, upper_red1), 
                                  cv2.inRange(hsv_image, lower_red2, upper_red2))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        roi_h, roi_w = roi_image.shape[:2]
        y_steps = range(10, roi_h - 10, 10)

        centers_g = self.get_line_centers(mask_green, y_steps)
        centers_r = self.get_line_centers(mask_red, y_steps)

        virtual_path_x = []
        virtual_path_y = []

        direction = self.get_parameter('roundabout_dir').get_parameter_value().string_value

        for y in y_steps:
            x_g = centers_g.get(y)
            x_r = centers_r.get(y)

            if x_g is not None and x_r is not None:
                if x_r > x_g:
                    alpha = 0.1
                    self.last_lane_width = alpha * (x_r - x_g) + (1 - alpha) * self.last_lane_width
                vx = (x_g + x_r) / 2.0
                virtual_path_x.append(vx)
                virtual_path_y.append(y)
            elif x_r is not None:
                vx = x_r - (self.last_lane_width / 2.0)
                virtual_path_x.append(vx)
                virtual_path_y.append(y)
            elif x_g is not None:
                vx = x_g + (self.last_lane_width / 2.0)
                virtual_path_x.append(vx)
                virtual_path_y.append(y)

        target_x = self.last_target_x
        lookahead_y = int(roi_h * 0.3)

        if len(virtual_path_y) >= 3:
            poly = np.polyfit(virtual_path_y, virtual_path_x, 2)
            
            target_x = np.polyval(poly, lookahead_y)
            self.last_target_x = target_x

            pts = []
            for y in range(0, roi_h, 5):
                x = int(np.polyval(poly, y))
                pts.append([x, y + roi_top])
            pts = np.array(pts, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], False, (0, 255, 255), 2)
            
        elif len(virtual_path_y) > 0:
            target_x = np.mean(virtual_path_x)
            self.last_target_x = target_x

        center_cam = roi_w / 2.0
        error_x = center_cam - target_x

        twist = Twist()
        kp_x = 0.005
        angular = float(error_x * kp_x)

        if len(virtual_path_y) == 0:
            twist.linear.x = 0.05
        else:
            twist.linear.x = max(0.05, 0.15 - 0.5 * abs(angular))

        twist.angular.z = angular
        self.publisher_.publish(twist)

        cv2.line(cv_image, (0, roi_top), (width, roi_top), (255, 0, 255), 2)
        lookahead_draw_y = roi_top + lookahead_y if len(virtual_path_y) >= 3 else roi_top + int(roi_h/2)
        cv2.circle(cv_image, (int(target_x), lookahead_draw_y), 7, (255, 0, 0), -1)

        cv2.imshow("Debug View", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()