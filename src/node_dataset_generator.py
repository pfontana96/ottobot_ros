#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, TimeSynchronizer

from collections import OrderedDict
from queue import Queue, Empty
from pathlib import Path
import json

import cv2


class DatasetGeneratorNode:

    def __init__(self, verbose: bool = False):
        rospy.init_node("dataset_generator", anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO))

        nodename = rospy.get_name()

        color_topic = rospy.get_param("{}/color_in".format(nodename), "{}/raw/color_image".format(nodename))
        depth_topic = rospy.get_param("{}/depth_in".format(nodename), "{}/raw/depth_image".format(nodename))

        output_dir = rospy.get_param("{}/output_dir".format(nodename))
        self._output_dir = Path(output_dir).resolve()

        if not self._output_dir.is_dir():
            self._output_dir.mkdir(parents=True, exist_ok=False)

        rospy.loginfo("Dataset will be generated at '{}'".format(str(self._output_dir)))

        rate = rospy.get_param("{}/rate".format(nodename), 10)
        self._timeout = 1 / (3 * rate)
        self._rate = rospy.Rate(rate)

        self._tss = TimeSynchronizer([Subscriber(color_topic, Image), Subscriber(depth_topic, Image)], 5)
        self._tss.registerCallback(self.callback)

        self._cv_bridge = CvBridge()

        self._queue = Queue(maxsize=15)

    def callback(self, color_msg: Image, depth_msg: Image):

        try:
            color_image = self._cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self._cv_bridge.imgmsg_to_cv2(depth_msg, "mono16")

            data = OrderedDict({
                "color_timestamp": color_msg.header.stamp,
                "depth_timestamp": color_msg.header.stamp,
                "color_image": color_image,
                "depth_image": depth_image
            })

            self._queue.put(data, block=True, timeout=self._timeout)

        except CvBridgeError as e:
            rospy.logerr(e)
            return

        rospy.loginfo("Color image: {} | Depth image: {}".format(color_image.shape, depth_image.shape))

    def run(self):

        color_dir = self._output_dir / "rgb"
        depth_dir = self._output_dir / "depth"
        groundtruth_file = self._output_dir / "groundtruth.json"

        color_dir.mkdir(exist_ok=True)
        depth_dir.mkdir(exist_ok=True)

        groundtruth = OrderedDict({})

        try:
            while not rospy.is_shutdown():

                try:
                    data = self._queue.get(block=False)

                except Empty:
                    continue

                color_filename = "{}.png".format(data["color_timestamp"])
                depth_filename = "{}.png".format(data["depth_timestamp"])

                cv2.imwrite(str(color_dir / color_filename), data["color_image"])
                cv2.imwrite(str(depth_dir / depth_filename), data["depth_image"])

                groundtruth.update({
                    str(data["color_timestamp"]): OrderedDict({
                        "rgb": "rgb/{}".format(color_filename),
                        "depth": "depth/{}".format(depth_filename)
                    })
                })

                self._rate.sleep()

        except Exception as e:
            rospy.logerr(e)

        finally:
            rospy.loginfo("Dumping groundtruth file upon exit at '{}'..".format(str(groundtruth_file)))
            with groundtruth_file.open("w") as fp:
                json.dump(groundtruth, fp, indent=3)

            rospy.loginfo("DONE")


if __name__ == "__main__":

    rs_camera_node = DatasetGeneratorNode()
    rs_camera_node.run()
