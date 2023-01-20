#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs
import numpy as np
import yaml
from  pathlib import Path
import cv2

from ottobot.rgbd_camera.realsensed435i import RealSenseD435i

# PyYaml workaround for indentation
class MyDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)

class RealSenseCameraNode:
    def __init__(self, verbose: bool = False):
        rospy.init_node("realsense_node", anonymous=True, log_level=(rospy.DEBUG if verbose else rospy.INFO))

        nodename = rospy.get_name()

        self._frame_id = rospy.get_param("{}/frame_id".format(nodename), "map")
        self._frame_rate = rospy.get_param("{}/frame_rate".format(nodename), 15) # Defaults to 15 fps
        self._compressed = rospy.get_param("{}/compress".format(nodename), False)
        self._ros_rate = rospy.Rate(self._frame_rate)

        self._calibration_filename = rospy.get_param("{}/calibration_filename".format(nodename), "camera_intrinsics")

        color_topic = rospy.get_param("{}/color_topic".format(nodename), "{}/raw/color_image".format(nodename))
        depth_topic = rospy.get_param("{}/color_depth_topic".format(nodename), "{}/raw/depth_image".format(nodename))

        self._rgb_pub = rospy.Publisher(color_topic, Image, queue_size=5)
        self._depth_pub = rospy.Publisher(depth_topic, Image, queue_size=5)

        if self._compressed:
            self._color_pub_compressed = rospy.Publisher("{}/compressed".format(color_topic), CompressedImage, queue_size=5)
            self._depth_pub_compressed = rospy.Publisher("{}/compressed".format(depth_topic), CompressedImage, queue_size=5)

        self._height = 480
        self._width = 848

        rospy.loginfo("Initializing camera node ({} fps): {}x{}..".format(self._frame_rate, self._height, self._width))

        self._camera = None                                       
        ctx = rs.context()
        devices = ctx.query_devices()

        dev_id = devices[0].get_info(rs.camera_info.serial_number)
        rospy.loginfo("Device found '{}' ({})".format(devices[0].get_info(rs.camera_info.name), dev_id))
        rospy.loginfo("Resetting..")
        devices[0].hardware_reset()
        rospy.loginfo("DONE")

        self._camera = RealSenseD435i(ctx, self._frame_rate, self._height, self._width, dev_id)

        self._cv_bridge = CvBridge()
    
    def run(self):

        try:
            rospy.loginfo("Starting streaming")

            # Get camera intrinsics and dump them in configuration file
            intrinsics = self._camera.intrinsics
            config_path = Path(__file__).resolve().parent.parent / "config"

            if config_path.exists() and config_path.is_dir():

                filename = (config_path / self._calibration_filename).with_suffix('.yaml')

                rospy.loginfo("Dumping camera intrinsics to '{}'..".format(str(filename)))
                with open(filename, 'w') as f:
                    yaml.dump(intrinsics, f, Dumper=MyDumper)

            else:
                rospy.logerr("Config directory does not exist: {}".format(str(config_path)))

            while not rospy.is_shutdown():
                # Read camera
                self._camera.poll()
                
                # Get time stamp
                now = rospy.get_rostime()

                # Prepare image messages
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_image = self._camera.depth_frame
                color_image = self._camera.color_frame

                if (depth_image is None) or (color_image is None):
                    rospy.logerr("Either color frame or depth frame is None..")
                    continue

                rospy.logdebug("depth: {} and color: {}".format(depth_image.shape, color_image.shape))
                self._send_images(depth_image=depth_image, color_image=color_image, time=now)

                if self._compressed:
                    # Compress images for visualization on Rviz on remote machine
                    self._send_compressed(depth_image=depth_image, color_image=color_image, time=now)

                # self._ros_rate.sleep()

        except Exception as e:
            rospy.logerr(e)

        finally:
            self._camera.shutdown()

    def _send_images(self, depth_image: np.ndarray, color_image: np.ndarray, time: int):
        # Convert images to ROS messages
        try:
            color_image_msg = self._cv_bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_image_msg = self._cv_bridge.cv2_to_imgmsg(depth_image, encoding="mono16")

        except CvBridgeError as e:
            rospy.logerr(e)
            return

        color_image_msg.header.stamp = time
        depth_image_msg.header.stamp = time

        color_image_msg.header.frame_id = self._frame_id
        color_image_msg.header.frame_id = self._frame_id


        self._rgb_pub.publish(color_image_msg)
        self._depth_pub.publish(depth_image_msg)

    def _send_compressed(self, depth_image: np.ndarray, color_image: np.ndarray, time: int):

        # Apply color map to depth image for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_image_msg = CompressedImage()
        depth_image_msg.header.stamp = time
        depth_image_msg.header.frame_id = self._frame_id
        depth_image_msg.format = "jpeg"
        depth_image_msg.data = np.array(cv2.imencode('.jpg', depth_colormap)[1]).tostring()

        color_image_msg = CompressedImage()
        color_image_msg.header.stamp = time
        color_image_msg.header.frame_id = self._frame_id
        color_image_msg.format = "jpeg"
        color_image_msg.data = np.array(cv2.imencode('.jpg', color_image)[1]).tostring()

        self._color_pub_compressed.publish(color_image_msg)
        self._depth_pub_compressed.publish(depth_image_msg)


if __name__ == "__main__":

    rs_camera_node = RealSenseCameraNode()
    rs_camera_node.run()

