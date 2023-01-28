#! /usr/bin/python3
import rospy
from std_msgs.msg import Int8

from board import SDA, SCL, D20, D26, D16, D19
from busio import I2C
import digitalio as dio
from adafruit_pca9685 import PCA9685


class DriverNode(object):
    """Controls Ottobot DC motors. Remember that when using the Jetson Nano, PWM pins should be set by hardware.
    Further info at: https://www.youtube.com/watch?v=eImDQ0PVu2Y
    """
    _MOTOR_PINS = {
        "IN1": dio.DigitalInOut(D20),
        "IN2": dio.DigitalInOut(D26),
        "IN3": dio.DigitalInOut(D19),
        "IN4": dio.DigitalInOut(D16),
    }

    def __init__(self, verbose: bool = False):

        rospy.init_node(
            'motor_driver_node', anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO)
        )

        nodename = rospy.get_name()

        in_topic = rospy.get_param("{}/in_topic".format(nodename), "key_teleop")
        duty_cycle_step = rospy.get_param("{}/duty_cycle_step".format(nodename), 0.30)

        # Pin setup
        # IN1 HIGH & IN2 LOW -> RIGHT FWD
        # IN1 LOW & IN2 HIGH -> RIGHT BWD
        # IN3 LOW & IN4 HIGH -> LEFT FWD
        # IN3 HIGH & IN4 LOW -> LEFT BWD

        rospy.logdebug("Initialising I2C bus and PCA9685 driver..")

        i2c_bus = I2C(SCL, SDA)

        self._pca = PCA9685(i2c_bus)
        self._pca.frequency = 60

        self._duty_step = duty_cycle_step
        self._duty_cycle_r = 0
        self._duty_cycle_l = 0

        self._pca.channels[0].duty_cycle = self._duty_cycle_r
        self._pca.channels[1].duty_cycle = self._duty_cycle_l

        rospy.logdebug("DONE")

        rospy.logdebug("Initialising PINs..")
        self._init_motor_pins()
        rospy.logdebug("DONE")
        
        rospy.logdebug("Subscribing to '{}'..".format(in_topic))
        self._sub = rospy.Subscriber(in_topic, Int8, self.callback)

    def _init_motor_pins(self):
        for pin in self._MOTOR_PINS.values():
            pin.direction = dio.Direction.OUTPUT
            pin.value = False

    def _shutdown(self):
        for pin in self._MOTOR_PINS.values():
            pin.deinit()

        self._pca.deinit()

    def run(self):
        try:
            rospy.spin()

        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupt requested")

        finally:
            rospy.loginfo("Resetting pins")
            self._shutdown()

    def callback(self, data):
        duty_cycle_r, duty_cycle_l, right_fwd, left_fwd = self._decode_msg(data.data, self._duty_step)
        rospy.loginfo("Right : {} || Left: {}".format(duty_cycle_r, duty_cycle_l))

        if duty_cycle_r != self._duty_cycle_r:
            if duty_cycle_r == 0:
                # Right motor not moving
                self._MOTOR_PINS["IN1"].value = False
                self._MOTOR_PINS["IN2"].value = False

            elif right_fwd:
                # Right motor moving forwards
                self._MOTOR_PINS["IN1"].value = True
                self._MOTOR_PINS["IN2"].value = False

            else:
                # Right motor moving backwards
                self._MOTOR_PINS["IN1"].value = False
                self._MOTOR_PINS["IN2"].value = True

            self._duty_cycle_r = duty_cycle_r

            rospy.logdebug("PWM Right: {}".format(self._duty_cycle_r))
            self._pca.channels[0].duty_cycle = int(65535 * self._duty_cycle_r)

        if duty_cycle_l != self._duty_cycle_l:
            if duty_cycle_l == 0:
                # Left motor not moving
                self._MOTOR_PINS["IN3"].value = False
                self._MOTOR_PINS["IN4"].value = False

            elif left_fwd:
                # Left motor moving forwards
                self._MOTOR_PINS["IN3"].value = True
                self._MOTOR_PINS["IN4"].value = False

            else:
                # Left motor moving backwards
                self._MOTOR_PINS["IN3"].value = False
                self._MOTOR_PINS["IN4"].value = True

            self._duty_cycle_l = duty_cycle_l
    
            rospy.logdebug("PWM Left: {}".format(self._duty_cycle_l))
            self._pca.channels[1].duty_cycle = int(65535 * self._duty_cycle_l)

    @staticmethod
    def _decode_msg(msg: int, duty_cycle_step: int = 0.2):
        """
        Decodes Message (Int8) into the PWM (% duty cycle) of the right and left motor signals 
        based on the following encoding:
            b'up dn lt rt
        Example: 1001 (9) => up & right
        """
        duty_cycle_r = 0
        duty_cycle_l = 0
        left_fwd = True

        if msg & (1 << 1):
            duty_cycle_r += duty_cycle_step

        elif msg & 1: # Turning left and right at the same time makes no sense
            duty_cycle_l += duty_cycle_step

        if msg & (1 << 3):
            duty_cycle_l += duty_cycle_step
            duty_cycle_r += duty_cycle_step

        elif msg & (1 << 2): # Going fwd and bwd at the same time makes no sense
            duty_cycle_l = -(duty_cycle_l + duty_cycle_step)
            duty_cycle_r = -(duty_cycle_r + duty_cycle_step)

        right_fwd =  duty_cycle_r >= 0
        left_fwd = duty_cycle_l >= 0

        duty_cycle_r = min(abs(duty_cycle_r), 1.0)
        duty_cycle_l = min(abs(duty_cycle_l), 1.0)

        return (duty_cycle_r, duty_cycle_l, right_fwd, left_fwd)
 

if __name__ == '__main__':

    verbose = True
    driver_node = DriverNode(verbose=verbose)

    driver_node.run()
