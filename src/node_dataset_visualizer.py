#! /home/ottobot/dev/ottobot_ws/venv/bin/python3

import json
from pathlib import Path

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

import numpy as np
import cv2
import tqdm

from dense_visual_odometry.camera_model import RGBDCameraModel
from dense_visual_odometry.utils.lie_algebra import Se3, So3


class DatasetVisualizer:
    def __init__(self):

        rospy.init_node("dataset_visualizer")

        nodename = rospy.get_name()

        self._rate = rospy.get_param("{}/rate".format(nodename), 5)  # Hz
        self._report_file = Path(rospy.get_param(
            "{}/report_file".format(nodename), "/home/ottobot/dev/dense-visual-odometry/data/report.json"
        ))

        self._frame_id = rospy.get_param(
            "{}/frame_id".format(nodename), "camera"
        )
        self._map_frame_id = rospy.get_param(
            "{}/map_frame_id".format(nodename), "map"
        )
        self._baselink_frame_id = rospy.get_param(
            "{}/baselink_frame_id".format(nodename), "base_link"
        )

        self._publish_pointcloud = rospy.get_param("{}/plot_pointcloud".format(nodename), False)
        if self._publish_pointcloud:
            topic_name = rospy.get_param("{}/topic_out".format(nodename), "pointcloud")

            self._pointcloud_publisher = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._robot_base_to_camera = Se3(
            So3(np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float32).reshape(4, 1), quat_repr="wxyz"),
            np.asarray([0.0, 0.0, 0.2], dtype=np.float32).reshape(3, 1)
        )

        self._load_data()

    def _load_data(self):
        with self._report_file.open("r") as fp:
            data = json.load(fp)

            camera_poses_list = data["estimated_transforms"]
            self._rgb_filepaths = data["rgb"]
            self._depth_filepaths = data["depth"]
            camera_intrinsics_file = Path(data["camera_intrinsics"])

        self._camera_model = RGBDCameraModel.load_from_yaml(camera_intrinsics_file)
        self._camera_poses = [None] * len(camera_poses_list)
        for i, (camera_pose) in tqdm.tqdm(
            enumerate(zip(camera_poses_list)), desc="Loading poses"
        ):

            se3 = Se3.from_se3(np.array(camera_pose).reshape(6, 1).astype(np.float32))
            self._camera_poses[i] = se3

    def _try_get_transform(self, from_frame_id: str, to_frame_id: str) -> Se3:
        """Tries to get transform from `from_frame_id` to `to_frame_id`

        Parameters
        ----------
        from_frame_id : str
            Source frame id.
        to_frame_id : str
            Destination frame id.

        Returns
        -------
        Se3
            Transform from `from_frame_id` to `to_frame_id`
        """

        timeout = 2  # [s]

        try:
            # NOTE: TF API is not so nice..
            # When referring to transform between coordinate frames (transforming the frames,
            # it is the inverse of the transform of data between the two frames)
            # See https://answers.ros.org/question/194046/the-problem-of-transformerlookuptransform/
            tf2_transform = self._tf_buffer.lookup_transform(
                from_frame_id, to_frame_id, rospy.Time(0), rospy.Duration(timeout)
            )

            tf_quat = tf2_transform.transform.rotation
            tf_tvec = tf2_transform.transform.translation

            transform = Se3(
                So3(np.array([tf_quat.w, tf_quat.x, tf_quat.y, tf_quat.z], dtype=np.float32).reshape(4, 1)),
                np.array([tf_tvec.x, tf_tvec.y, tf_tvec.z], dtype=np.float32).reshape(3, 1)
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Lookup for transform from '{}' to '{}' failed with '{}' within {}s. Using Identity..".format(
                self._frame_id, self._baselink_frame_id, e, timeout
            ))
            transform = Se3.identity()

        return transform

    def run(self):

        rate = rospy.Rate(self._rate)

        baselink_to_camera = self._try_get_transform(self._baselink_frame_id, self._frame_id)

        accumulated_transform = Se3.identity() * baselink_to_camera

        for i, (estimated_transform, rgb_path, depth_path) in enumerate(
            zip(self._camera_poses, self._rgb_filepaths, self._depth_filepaths)
        ):

            if rospy.is_shutdown():
                break

            stamp = rospy.Time.now()

            accumulated_transform = accumulated_transform * estimated_transform

            robot_pose = accumulated_transform * baselink_to_camera.inverse()

            quat = robot_pose.so3.quat.flatten().tolist()
            tvec = robot_pose.tvec.flatten().tolist()

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = stamp
            transform_stamped.header.frame_id = self._map_frame_id
            transform_stamped.child_frame_id = (
                self._frame_id if self._baselink_frame_id is None else self._baselink_frame_id
            )
            transform_stamped.transform.translation.x = tvec[0]
            transform_stamped.transform.translation.y = tvec[1]
            transform_stamped.transform.translation.z = tvec[2]
            transform_stamped.transform.rotation.w = quat[0]
            transform_stamped.transform.rotation.x = quat[1]
            transform_stamped.transform.rotation.y = quat[2]
            transform_stamped.transform.rotation.z = quat[3]

            self._tf_broadcaster.sendTransform(transform_stamped)

            if self._publish_pointcloud:

                rgb_image = cv2.cvtColor(cv2.imread(rgb_path, cv2.IMREAD_ANYCOLOR), cv2.COLOR_BGR2RGB)
                depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

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
