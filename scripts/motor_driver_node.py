#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int8

from RPi import GPIO


class DriverNode(object):
    _PIN_MAP = {
        "ENA": 33,
        "IN1": 36,
        "IN2": 31,
        "ENB": 32,
        "IN3": 35,
        "IN4": 37,
    }

    def __init__(self, in_topic: str, verbose: bool = False):

        rospy.init_node(
            'motor_driver_node', anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO)
        )

        rospy.logdebug("Subscribing to '{}'".format(in_topic))
        self._sub = rospy.Subscriber(in_topic, Int8, self.callback)

        # Pin setup
        # IN1 HIGH & IN2 LOW -> RIGHT FWD
        # IN1 LOW & IN2 HIGH -> RIGHT BWD
        # IN3 LOW & IN4 HIGH -> LEFT FWD
        # IN3 HIGH & IN4 LOW -> LEFT BWD

        GPIO.setup(self._PIN_MAP["IN1"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._PIN_MAP["IN2"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._PIN_MAP["IN3"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._PIN_MAP["IN4"], GPIO.OUT, initial=GPIO.HIGH)

        GPIO.setup(self._PIN_MAP["ENA"], GPIO.OUT)
        GPIO.setup(self._PIN_MAP["ENB"], GPIO.OUT)

        self._motor_r_pwm = GPIO.PWM(self._PIN_MAP["ENA"], 100)
        self._motor_l_pwm = GPIO.PWM(self._PIN_MAP["ENB"], 100)

    def run(self):
        rospy.spin()

    def callback(self, data):
        duty_cycle_r, duty_cycle_l = self._decode_msg(data.data)
        rospy.loginfo("Right : {} || Left: {}".format(duty_cycle_r, duty_cycle_l))

    @staticmethod
    def _decode_msg(msg: int):
        """
        Decodes Message (Int8) into the PWM (% duty cycle) of the right and left motor signals 
        based on the following encoding:
            b'up dn lt rt
        Example: 1001 (9) => up & right
        """
        duty_cycle_r = 0
        duty_cycle_l = 0
        d_duty = 30 # step of duty cycle [%]

        if msg & (1<<3):
            duty_cycle_l += d_duty
            duty_cycle_r += d_duty

        elif msg & (1<<2): # Going fwd and bwd at the same time makes no sense
            duty_cycle_l -= d_duty
            duty_cycle_r -= d_duty

        if msg & (1<<1):
            duty_cycle_l += d_duty
            duty_cycle_r -= d_duty

        elif msg & 1: # Turning left and right at the same time makes no sense
            duty_cycle_l -= d_duty
            duty_cycle_r += d_duty

        # A negative duty cycle means a change in direction of motor
        for duty in (duty_cycle_r, duty_cycle_l):
            if duty < 0:
                # Change motor's direction!
                duty = abs(duty)

        return (duty_cycle_r, duty_cycle_l)
 

if __name__ == '__main__':

    namespace = rospy.get_namespace()
    topic_name = rospy.get_param("in_topic", "key_teleop")
    verbose = rospy.get_param("verbose", True)

    if namespace != "":
        topic_name = namespace + topic_name

    driver_node = DriverNode(in_topic=topic_name, verbose=verbose)

    try:
        driver_node.run()
    except rospy.ROSInterruptException:
        pass