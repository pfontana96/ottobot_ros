#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int8

def decodeMsg(msg):
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


class DriverNode(object):
    def __init__(self):
        rospy.init_node('driver_node')
        rospy.Subscriber('ottobot/key_teleop', Int8, self.callback)

    def run(self):
        rospy.spin()

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        duty_cycle_r, duty_cycle_l = decodeMsg(data.data)
        rospy.loginfo("Right : {} || Left: {}".format(duty_cycle_r, duty_cycle_l))
 
if __name__ == '__main__':
    driver_node = DriverNode()
    try:
        driver_node.run()
    except rospy.ROSInterruptException:
        pass