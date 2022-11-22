#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8

import sys
from select import select
import threading

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class TerminalMonitorThread(threading.Thread):

    def __init__(self, key_timeout: float):
        super(TerminalMonitorThread, self).__init__()

        self._lock = threading.Lock()
        self._done = False

        self._last_keys = dict()  # Using dictionary as an Ordered set

        self._terminal_settings = self._save_terminal_settings()
        self._timeout = key_timeout

    def get_last_key(self):

        with self._lock:
            last_keys = self._last_keys.copy()
            self._last_keys.clear()

        # rospy.logdebug("Collected keys {}\r\n".format(last_keys.keys()))
        try:
            last_keys.pop("")  # Remove empty keys that might have been collected
        
        except KeyError:
            pass

        # If there is nothing left, then return empty string, else return last component
        if not last_keys:
            key = ""
        
        else:
            key = list(last_keys.keys())[-1]

        return key

    def stop(self):
        self._done = True
        self.join()
        self._restore_terminal_settings(self._terminal_settings)

    def run(self):
        while not self._done:

            key = self._get_key(self._terminal_settings, self._timeout)

            with self._lock:
                self._last_keys.update({key: None})

    @staticmethod
    def _get_key(settings, timeout):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    @staticmethod
    def _save_terminal_settings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    @staticmethod
    def _restore_terminal_settings(old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# Message trame
# b'up dn lt rt
# Example: 1001 => up & right
class TeleopNode:
    _INFO_MSG = """
    To control the robot please use:
    \tq w e
    \ta   d
    \tz x c
    """
    def __init__(self, rate: float, topic_name: str):
        rospy.init_node('teleop_driver', anonymous=True, log_level=rospy.DEBUG)
        self._pub = rospy.Publisher(topic_name, Int8, queue_size=10) # "key" is the publisher name
        self._rate = rospy.Rate(rate)

        key_timeout = 1 / (5 * rate)  # x10 faster than publisher Node
        self._terminal_monitor_thread = TerminalMonitorThread(key_timeout=key_timeout)

        self._key_bindings = {
            "q": 0b1010,
            "w": 0b1000,
            "e": 0b1001,
            "a": 0b0010,
            "d": 0b0001,
            "z": 0b0110,
            "x": 0b0100,
            "c": 0b0101,
            "": 0
        }
    
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self._pub.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self._pub.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def run(self):
        self._terminal_monitor_thread.start()

        try:
            while not rospy.is_shutdown():

                key = self._terminal_monitor_thread.get_last_key()
                # rospy.loginfo("{}\r\n".format(key))

                if (key == '\x03'):
                    break

                msg = self._encode_msg(key=key)
                self._pub.publish(msg)              
                self._rate.sleep()

        finally:
            self._terminal_monitor_thread.stop()

    def _encode_msg(self, key: str):
        msg = 0
        if key in self._key_bindings:
            msg = self._key_bindings[key]
        
        else:
            rospy.logdebug(self._INFO_MSG)

        return msg


if __name__ == '__main__':

    rate = rospy.get_param("rate", 10.0)
    namespace = rospy.get_namespace()
    topic_name = rospy.get_param("out_topic", "key_teleop")

    if namespace != "":
        topic_name = namespace + topic_name

    teleop_driver = TeleopNode(rate=rate, topic_name=topic_name)

    try:
        teleop_driver.run()
    except rospy.ROSInterruptException:
        pass
