#! /home/ottobot/dev/ottobot_ws/venv/bin/python3

import json
from pathlib import Path

import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np
import cv2

from dense_visual_odometry.camera_model import RGBDCameraModel
from dense_visual_odometry.utils.lie_algebra import Se3


class DatasetVisualizer:
    def __init__(self):

        rospy.init_node("dataset_visualizer")

        nodename = rospy.get_name()

        self._rate = rospy.get_param("{}/rate".format(nodename), 5)  # Hz
        self._report_file = rospy.get_param(
            "{}/report_file".format(nodename), "/home/ottobot/dev/dense-visual-odometry/data/report.json"
        )

        self._publish_pointcloud = rospy.get_param("{}/plot_pointcloud".format(nodename), False)
        if self._publish_pointcloud:
            topic_name = rospy.get_param("{}/topic_out".format(nodename), "pointcloud")

            self._pointcloud_publisher = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

        self._tf_broadcaster = tf.TransformBroadcaster()

        self._load_data()

    def _load_data(self):
        with open(self._report_file, "r") as fp:
            data = json.load(fp)

            camera_poses_list = data["estimated_transformations"]
            rgb_filepaths = data["rgb"]
            depth_filepaths = data["depth"]
            camera_intrinsics_file = Path(data["camera_intrinsics"])

        self._camera_model = RGBDCameraModel.load_from_yaml(camera_intrinsics_file)
        self._camera_poses = [None] * len(camera_poses_list)
        self._rgb_images = [None] * len(camera_poses_list)
        self._depth_images = [None] * len(camera_poses_list)
        for i, (camera_pose, rgb_path, depth_path) in enumerate(zip(camera_poses_list, rgb_filepaths, depth_filepaths)):

            se3 = Se3.from_se3(np.array(camera_pose).reshape(6, 1).astype(np.float32))
            self._camera_poses[i] = {
                "position": {
                    "x": se3.tvec[0, 0],
                    "y": se3.tvec[1, 0],
                    "z": se3.tvec[2, 0]
                },
                "orientation": {
                    "w": se3.so3.quat[0],
                    "x": se3.so3.quat[1],
                    "y": se3.so3.quat[2],
                    "z": se3.so3.quat[3]
                }
            }

            if self._publish_pointcloud:
                self._rgb_images[i] = cv2.cvtColor(cv2.imread(rgb_path, cv2.IMREAD_ANYCOLOR), cv2.COLOR_BGR2RGB)
                self._depth_images[i] = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

    def run(self):

        rate = rospy.Rate(self._rate)

        for i, (camera_pose, rgb_image, depth_image) in enumerate(
            zip(self._camera_poses, self._rgb_images, self._depth_images)
        ):

            stamp = rospy.Time.now()

            position = camera_pose["position"]
            orientation = camera_pose["orientation"]

            self._tf_broadcaster.sendTransform(
                (position["x"], position["y"], position["z"]),
                (orientation["x"], orientation["y"], orientation["z"], orientation["w"]),
                stamp,
                "camera",
                "map"
            )

            if self._publish_pointcloud:
                # Create point cloud
                pointcloud_xyz, valid_pixel_mask = self._camera_model.deproject(
                    depth_image=depth_image, return_mask=True
                )
                pointcloud_xyz = pointcloud_xyz.T
                N = pointcloud_xyz.shape[0]
                pointcloud_color = rgb_image[valid_pixel_mask] / 255.0

                data = np.hstack((
                    pointcloud_xyz[:, :3], pointcloud_color
                )).astype(np.float32).tobytes()

                pointcloud_msg = PointCloud2()
                pointcloud_msg.header = Header()
                pointcloud_msg.header.stamp = stamp
                pointcloud_msg.header.frame_id = "camera"

                # Fill in pointcloud_msg fields, including point data
                itemsize = np.dtype(np.float32).itemsize
                ros_type = PointField.FLOAT32
                pointcloud_msg.fields = [
                    PointField(name=n, offset=i * itemsize, datatype=ros_type, count=1) for i, n in enumerate("xyzrgb")
                ]

                pointcloud_msg.is_bigendian = False
                pointcloud_msg.point_step = (itemsize * 6)
                pointcloud_msg.row_step = pointcloud_msg.point_step * N
                pointcloud_msg.is_dense = False
                pointcloud_msg.height = 1
                pointcloud_msg.width = N

                pointcloud_msg.data = data

                self._pointcloud_publisher.publish(pointcloud_msg)

            rospy.loginfo("Done for frame: {}".format(i + 1))

            rate.sleep()

        rospy.loginfo("Finished")
        rospy.spin()


if __name__ == '__main__':

    camera_pose_publisher = DatasetVisualizer()
    camera_pose_publisher.run()
