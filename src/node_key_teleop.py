#!/usr/bin/python3
import rospy
from std_msgs.msg import Int8

import sys
from select import select
import threading
from collections import OrderedDict

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class TerminalMonitorThread(threading.Thread):

    _VALID_KEYS = ["q", "w", "e", "a", "d", "z", "x", "c", "", '\x03']

    def __init__(self, key_timeout: float):
        super(TerminalMonitorThread, self).__init__()

        self._lock = threading.Lock()
        self._done = False

        self._last_keys = OrderedDict()  # Using dictionary as an Ordered set

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

        # If there is nothing left, then return empty string, else return most retrieved key
        if not last_keys:
            key = ""

        else:
            try:
                if last_keys['\x03'] >= 0:
                    key = '\x03'

            except KeyError:
                key = max(last_keys, key=last_keys.get)

        return key

    def stop(self):
        self._done = True
        self.join()
        self._restore_terminal_settings(self._terminal_settings)

    def run(self):
        while not self._done:

            key = self._get_key(self._terminal_settings, self._timeout)

            if key in self._VALID_KEYS:
                with self._lock:
                    if key not in self._last_keys.keys():
                        self._last_keys.update({key: 0})

                    self._last_keys[key] += 1

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

    def __init__(self, verbose: bool = False):

        rospy.init_node(
            'teleop_driver', anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO)
        )

        nodename = rospy.get_name()

        rate = rospy.get_param("{}/rate".format(nodename), 10.0)
        topic_name = rospy.get_param("{}/out_topic".format(nodename), "key_teleop")

        self._pub = rospy.Publisher(topic_name, Int8, queue_size=10)  # "key" is the publisher name
        self._rate = rospy.Rate(rate)

        key_timeout = 1 / (10 * rate)  # x10 faster than publisher Node
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

    verbose = True
    teleop_driver = TeleopNode(verbose=verbose)

    try:
        teleop_driver.run()

    except rospy.ROSInterruptException:
        pass
