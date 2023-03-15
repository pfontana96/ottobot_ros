#! /home/ottobot/dev/ottobot_ws/venv/bin/python3

import json
from pathlib import Path
from typing import Union

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

import numpy as np
import cv2
import tqdm
from scipy.spatial.transform import Rotation
import sophus as so
import yaml


class RGBDCameraPinholeModel:

    # Keywords for loading camera model from a config file
    INTRINSICS_KEYWORD = "intrinsics"
    DEPTH_SCALE_KEYWORD = "depth_scale"

    def __init__(self, intrinsics: np.ndarray, depth_scale: float):
        """
            Creates a RGBDCameraPinholeModel instance
        Parameters
        ----------
        intrinsics : np.ndarray
            3x3 instrinsics matrix
                [fx  s cx]
                [ 0 fy cy]
                [ 0  0  1]
            with:
                fx, fy: Focal lengths for the sensor in X and Y dimensions
                s: Any possible skew between the sensor axes caused by sensor not being perpendicular from optical axis
                cx, cy: Image center expressed in pixel coordinates
        depth_scale : float
            Scale (multiplication factor) used to convert from the sensor Digital Number to meters
        """
        assertion_message = f"Expected a 3x3 'intrinsics', got {intrinsics.shape} instead"
        assert intrinsics.shape == (3, 3), assertion_message
        assertion_message = "Expected 'scale' to be a positive floating point, got '{:.3f}' instead".format(depth_scale)
        assert depth_scale >= 0.0, assertion_message

        # Store calibration matrix as a 3x4 matrix
        self._intrinsics = np.zeros((3, 4), dtype=np.float32)
        self._intrinsics[:3, :3] = intrinsics
        self.depth_scale = depth_scale

    @classmethod
    def load_from_yaml(cls, filepath: Union[str, Path]):
        """
            Loads RGBDCameraModel instance from a YAML file
        Parameters
        ----------
        filepath : Path
            Path to configuration file
        Returns
        -------
        camera_model : RGBDCameraModel | None
            Camera model loaded from file if possible, None otherwise
        """

        if not filepath.exists():
            raise FileNotFoundError("Could not find configuration file '{}'".format(str(filepath)))

        with filepath.open("r") as fp:
            data = yaml.load(fp, yaml.Loader)

        try:
            camera_matrix = np.array(data[cls.INTRINSICS_KEYWORD], dtype=np.float32).reshape(3, 3)
            depth_scale = data[cls.DEPTH_SCALE_KEYWORD]

        except KeyError as e:
            raise ValueError("Could not find mandatory key '{}' on RGBDCameraPinholeModel config file at '{}'".format(
                e, str(filepath)
            ))

        return cls(camera_matrix, depth_scale)

    def deproject(self, depth_image: np.ndarray, return_mask: bool = False):
        """
            Deprojects a depth image into the camera reference frame
        Parameters
        ----------
        depth_image : np.ndarray
            Depth image (with invalid pixels defined with the value 0)
        return_mask : bool
            if True, then a bolean mask is returned with valid pixels
        Returns
        -------
        pointcloud : np.ndarray
            3D Point (4xN) coordinates of projected points in Homogeneous coordinates (i.e x, y, z, 1)
        mask : np.ndarray, optional
            Boolean mask with the same shape as `depth_image` with True on valid pixels and false on non valid.
        """
        height, width = depth_image.shape

        # Remove invalid points
        mask = depth_image != 0.0
        mask = mask.reshape(-1)

        z = depth_image.reshape(-1) * self.depth_scale
        z = z[mask].astype(np.float32)

        # Compute sensor grid
        # TODO: Use a more efficient way of creating pointcloud -> Several pixels values are repeated. See `sparse`
        # parameter of `np.meshgrid`
        x_pixel, y_pixel = np.meshgrid(
            np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32), copy=False
        )
        x_pixel = x_pixel.reshape(-1)
        y_pixel = y_pixel.reshape(-1)

        x_pixel = x_pixel[mask]
        y_pixel = y_pixel[mask]

        # Map from pixel position to 3d coordinates using camera matrix (inverted)
        # Get x, y points w.r.t camera reference frame (still not multiply by the depth)
        points = np.dot(np.linalg.inv(self._intrinsics[:3, :3]), np.vstack((x_pixel, y_pixel, np.ones_like(z))))

        pointcloud = np.vstack((points[0, :] * z, points[1, :] * z, z, np.ones_like(z)))

        if return_mask:
            return (pointcloud, mask.reshape(depth_image.shape))

        return pointcloud


def SE3_from_quat_trans(quat: np.ndarray, trans: np.ndarray):
    """Loads a Sophus SE3 objetc from a quaternion and a translation vector

    Parameters
    ----------
    quat : np.ndarray
        Quaternion
    trans : np.ndarray
        Translation

    Returns
    -------
    SE3 : SE3
        SE3 group 4x4 matrix
    """
    rotmat = Rotation.from_quat(quat).as_matrix()
    return so.SE3(rotmat, trans.reshape(3, 1))


class DatasetVisualizerNode:
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
        self._absolute_transforms = rospy.get_param("{}/absolute", True)

        self._publish_pointcloud = rospy.get_param("{}/plot_pointcloud".format(nodename), False)
        if self._publish_pointcloud:
            topic_name = rospy.get_param("{}/topic_out".format(nodename), "pointcloud")

            self._pointcloud_publisher = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._load_data()

    def _load_data(self):
        with self._report_file.open("r") as fp:
            data = json.load(fp)

            if not self._absolute_transforms:
                camera_poses_list = data["estimated_transforms"]
            else:
                camera_poses_list = data["estimated_poses"]

            self._rgb_images = data["rgb"]
            self._depth_images = data["depth"]
            camera_intrinsics_file = Path(data["camera_intrinsics"])

        self._camera_model = RGBDCameraPinholeModel.load_from_yaml(camera_intrinsics_file)
        self._camera_poses = [None] * len(camera_poses_list)
        for i, camera_pose in tqdm.tqdm(enumerate(camera_poses_list), desc="Loading poses"):

            self._camera_poses[i] = SE3_from_quat_trans(np.array(camera_pose[3:]), np.array(camera_pose[:3]))

            if self._publish_pointcloud:
                self._rgb_images[i] = cv2.cvtColor(
                    cv2.imread(self._rgb_images[i], cv2.IMREAD_ANYCOLOR), cv2.COLOR_BGR2RGB
                )
                self._depth_images[i] = cv2.imread(self._depth_images[i], cv2.IMREAD_UNCHANGED)

    def _try_get_transform(self, from_frame_id: str, to_frame_id: str) -> so.SE3:
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

            transform = SE3_from_quat_trans(
                np.array([tf_quat.x, tf_quat.y, tf_quat.z, tf_quat.w], dtype=np.float32),
                np.array([tf_tvec.x, tf_tvec.y, tf_tvec.z], dtype=np.float32).reshape(3, 1)
            )

            # rospy.loginfo("\n{}".format(transform.exp()))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Lookup for transform from '{}' to '{}' failed with '{}' within {}s. Using Identity..".format(
                self._frame_id, self._baselink_frame_id, e, timeout
            ))
            transform = so.SE3()

        return transform

    def run(self):

        rate = rospy.Rate(self._rate)

        baselink_to_camera = self._try_get_transform(self._baselink_frame_id, self._frame_id)

        accumulated_transform = so.SE3()

        for i, (estimated_transform, rgb, depth) in enumerate(
            zip(self._camera_poses, self._rgb_images, self._depth_images)
        ):

            if rospy.is_shutdown():
                break

            stamp = rospy.Time.now()

            if not self._absolute_transforms:
                accumulated_transform = accumulated_transform * estimated_transform.inverse()

            else:
                accumulated_transform = estimated_transform

            robot_pose = baselink_to_camera * accumulated_transform

            quat = Rotation.from_matrix(robot_pose.so3().matrix()).as_quat().flatten().tolist()
            tvec = robot_pose.translation().flatten().tolist()

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = stamp
            transform_stamped.header.frame_id = self._map_frame_id
            transform_stamped.child_frame_id = "camera"
            transform_stamped.transform.translation.x = tvec[0]
            transform_stamped.transform.translation.y = tvec[1]
            transform_stamped.transform.translation.z = tvec[2]
            transform_stamped.transform.rotation.w = quat[3]
            transform_stamped.transform.rotation.x = quat[0]
            transform_stamped.transform.rotation.y = quat[1]
            transform_stamped.transform.rotation.z = quat[2]

            self._tf_broadcaster.sendTransform(transform_stamped)

            if self._publish_pointcloud:

                # Create point cloud
                pointcloud_xyz, valid_pixel_mask = self._camera_model.deproject(
                    depth_image=depth, return_mask=True
                )
                pointcloud_xyz = pointcloud_xyz.T
                N = pointcloud_xyz.shape[0]
                pointcloud_color = rgb[valid_pixel_mask] / 255.0

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

    camera_pose_publisher = DatasetVisualizerNode()
    camera_pose_publisher.run()
