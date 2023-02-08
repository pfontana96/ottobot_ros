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
        self._report_file = rospy.get_param("{}/report_file".format(nodename), "/home/ottobot/dev/dense-visual-odometry/data/report.json")
        topic_name = rospy.get_param("{}/topic_out".format(nodename), "pointcloud")

        self._pointcloud_publisher = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._camera_poses, self._rgb_images, self._depth_images, self._camera_model = self.load_data()

    def load_data(self):
        with open(self._report_file, "r") as fp:
            data = json.load(fp)

            camera_poses_list = data["estimated_transformations"]
            rgb_filepaths = data["rgb"]
            depth_filepaths = data["depth"]
            camera_intrinsics_file = Path(data["camera_intrinsics"])
        
        camera_model = RGBDCameraModel.load_from_yaml(camera_intrinsics_file)
        camera_poses = [None] * len(camera_poses_list)
        rgb_images = [None] * len(camera_poses_list)
        depth_images = [None] * len(camera_poses_list)
        for i, (camera_pose, rgb_path, depth_path) in enumerate(zip(camera_poses_list, rgb_filepaths, depth_filepaths)):

            se3 = Se3.from_se3(np.array(camera_pose).reshape(6, 1).astype(np.float32))
            camera_poses[i] = {
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
            rgb_images[i] = cv2.cvtColor(cv2.imread(rgb_path, cv2.IMREAD_ANYCOLOR), cv2.COLOR_BGR2RGB)
            depth_images[i] = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        return camera_poses, rgb_images, depth_images, camera_model

    def start(self):
        rate = rospy.Rate(self._rate)
        for i, (camera_pose, rgb_image, depth_image) in enumerate(zip(self._camera_poses, self._rgb_images, self._depth_images)):

            stamp = rospy.Time.now()

            position = camera_pose["position"]
            orientation = camera_pose["orientation"]

            self._tf_broadcaster.sendTransform((position["x"], position["y"], position["z"]),
                                               (orientation["x"], orientation["y"], orientation["z"], orientation["w"]),
                                               rospy.Time.now(),
                                               "camera",
                                               "map")

            # Create point cloud
            pointcloud_xyz, valid_pixel_mask = self._camera_model.deproject(depth_image=depth_image, return_mask=True)
            pointcloud_color = rgb_image[valid_pixel_mask].T

            pointcloud_msg = PointCloud2()
            pointcloud_msg.header = Header()
            pointcloud_msg.header.stamp = rospy.Time.now()
            pointcloud_msg.header.frame_id = "camera"

            # Fill in pointcloud_msg fields, including point data
            pointcloud_msg.fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("r", 12, PointField.FLOAT32, 1),
                PointField("g", 16, PointField.FLOAT32, 1),
                PointField("b", 20, PointField.FLOAT32, 1)
            ]
            pointcloud_msg.is_bigendian = False
            pointcloud_msg.point_step = 24
            pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_xyz.shape[1]
            pointcloud_msg.is_dense = True
            pointcloud_msg.height = 1
            pointcloud_msg.width = pointcloud_xyz.shape[1]

            pointcloud_msg.data = np.hstack((pointcloud_xyz[:3, :], pointcloud_color)).tobytes()

            self._pointcloud_publisher.publish(pointcloud_msg)

            rospy.loginfo("Done for frame: {}".format(i + 1))

            rate.sleep()

if __name__ == '__main__':
    camera_pose_publisher = DatasetVisualizer()
    camera_pose_publisher.start()
    rospy.spin()