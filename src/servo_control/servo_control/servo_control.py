import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy 
import Adafruit_PCA9685

class ServoControl(Node):

    def __init__(self):
        super().__init__('servo_control')
        self.subscriber = self.create_subscription(Joy,'joy',self.servo_callback,10)

        # default steering value (90 degree)
        self.steering_default = 375
        self.speed_neutral = 400

        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.pwm.set_pwm_freq(60)

        self.pwm.set_pwm(0, 0, int(self.speed_neutral))
        self.pwm.set_pwm(1, 0, int(self.steering_default))

    def servo_callback(self,msg):
        self.current_steering = -(msg.axes[0] * 225) + self.steering_default
        self.current_speed = ((-(msg.axes[5] - 1) / 2) * 200 + ((msg.axes[4] -1) / 2) * 200) + self.speed_neutral
        self.pwm.set_pwm(0, 0, int(self.current_speed))
        self.pwm.set_pwm(1, 0, int(self.current_steering))


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControl()
    rclpy.spin(servo_control_node)

    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
