#!/usr/bin/python3
import rospy
from std_msgs.msg import String, Int8

from Jetson import GPIO


class DriverNode(object):
    """Controls Ottobot DC motors. Remember that when using the Jetson Nano, PWM pins should be set by hardware.
    Further info at: https://www.youtube.com/watch?v=eImDQ0PVu2Y
    """
    _PIN_MAP = {
        "ENA": 33,
        "IN1": 36,
        "IN2": 31,
        "ENB": 32,
        "IN3": 35,
        "IN4": 37,
    }

    def __init__(self, verbose: bool = False):

        rospy.init_node(
            'motor_driver_node', anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO)
        )

        nodename = rospy.get_name()

        in_topic = rospy.get_param("{}/in_topic".format(nodename), "key_teleop")
        duty_cycle_step = rospy.get_param("{}/duty_cycle_step".format(nodename), 20)

        # Pin setup
        # IN1 HIGH & IN2 LOW -> RIGHT FWD
        # IN1 LOW & IN2 HIGH -> RIGHT BWD
        # IN3 LOW & IN4 HIGH -> LEFT FWD
        # IN3 HIGH & IN4 LOW -> LEFT BWD

        rospy.logdebug("Board: '{}'".format(GPIO.model))

        rospy.logdebug("Initialising PINs..")
        
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self._PIN_MAP["IN1"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._PIN_MAP["IN2"], GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self._PIN_MAP["IN3"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._PIN_MAP["IN4"], GPIO.OUT, initial=GPIO.HIGH)

        GPIO.setup(self._PIN_MAP["ENA"], GPIO.OUT)
        GPIO.setup(self._PIN_MAP["ENB"], GPIO.OUT)

        self._motor_r_pwm = GPIO.PWM(self._PIN_MAP["ENA"], 100)
        self._motor_l_pwm = GPIO.PWM(self._PIN_MAP["ENB"], 100)

        rospy.logdebug("Starting PWMs..")
        self._duty_step = duty_cycle_step
        self._duty_cycle_r = 0
        self._duty_cycle_l = 0

        self._motor_l_pwm.start(45)
        self._motor_r_pwm.start(45)

        rospy.logdebug("Subscribing to '{}'..".format(in_topic))
        self._sub = rospy.Subscriber(in_topic, Int8, self.callback)

    def run(self):
        try:
            rospy.spin()

        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupt requested")

        finally:
            rospy.loginfo("Resetting pins")
            self._motor_l_pwm.stop()
            self._motor_r_pwm.stop()
            GPIO.cleanup()

    def callback(self, data):
        duty_cycle_r, duty_cycle_l = self._decode_msg(data.data, self._duty_step)
        rospy.loginfo("Right : {} || Left: {}".format(duty_cycle_r, duty_cycle_l))

        if duty_cycle_r != self._duty_cycle_r:
            if duty_cycle_r == 0:
                # Right motor not moving
                GPIO.output(self._PIN_MAP["IN1"], GPIO.LOW)
                GPIO.output(self._PIN_MAP["IN2"], GPIO.LOW)

            elif duty_cycle_r < 0:
                # Right motor moving backwards
                GPIO.output(self._PIN_MAP["IN1"], GPIO.LOW)
                GPIO.output(self._PIN_MAP["IN2"], GPIO.HIGH)

            else:
                # Right motor moving forwards
                GPIO.output(self._PIN_MAP["IN1"], GPIO.HIGH)
                GPIO.output(self._PIN_MAP["IN2"], GPIO.LOW)

            self._duty_cycle_r = duty_cycle_r
            rospy.logdebug("PWM Right: {}".format(self._duty_cycle_r))
            self._motor_r_pwm.ChangeDutyCycle(abs(self._duty_cycle_r))

        if duty_cycle_l != self._duty_cycle_l:
            if duty_cycle_l == 0:
                # Right motor not moving
                GPIO.output(self._PIN_MAP["IN3"], GPIO.LOW)
                GPIO.output(self._PIN_MAP["IN4"], GPIO.LOW)

            elif duty_cycle_l < 0:
                # Right motor moving backwards
                GPIO.output(self._PIN_MAP["IN3"], GPIO.HIGH)
                GPIO.output(self._PIN_MAP["IN4"], GPIO.LOW)

            else:
                # Right motor moving forwards
                GPIO.output(self._PIN_MAP["IN3"], GPIO.LOW)
                GPIO.output(self._PIN_MAP["IN4"], GPIO.HIGH)

            self._duty_cycle_l = duty_cycle_l
            rospy.logdebug("PWM Left: {}".format(self._duty_cycle_l))
            self._motor_l_pwm.ChangeDutyCycle(abs(self._duty_cycle_l))

    @staticmethod
    def _decode_msg(msg: int, duty_cycle_step: int = 20):
        """
        Decodes Message (Int8) into the PWM (% duty cycle) of the right and left motor signals 
        based on the following encoding:
            b'up dn lt rt
        Example: 1001 (9) => up & right
        """
        duty_cycle_r = 0
        duty_cycle_l = 0

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

        return (duty_cycle_r, duty_cycle_l)
 

if __name__ == '__main__':

    verbose = True
    driver_node = DriverNode(verbose=verbose)

    driver_node.run()
