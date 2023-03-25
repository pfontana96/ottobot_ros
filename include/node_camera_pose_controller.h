#ifndef CAMERA_POSE_CONTROLLER_H
#define CAMERA_POSE_CONTROLLER_H

// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// Standard
#include <limits>
#include <thread>
#include <stdexcept>

// visual-odometry
#include <core/DenseVisualOdometry.h>
#include <utils/types.h>
#include <utils/YAMLLoader.h>

namespace otto{
    class CameraPoseControllerNode {

        using Image = sensor_msgs::Image;
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
        using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

        public:

            void run();
            void callback(  const sensor_msgs::ImageConstPtr& color, 
                            const sensor_msgs::ImageConstPtr& depth);

            CameraPoseControllerNode();
            ~CameraPoseControllerNode();

        private:
            // Attributes
            ros::NodeHandle nh_;
            ros::Rate rate;

            bool found_camera_to_baselink_, camera_info_set_;
            // bool pointcloud;

            message_filters::Subscriber<Image> color_sub, depth_sub;
            Synchronizer sync;

            vo::core::DenseVisualOdometry dvo;

            tf2_ros::TransformBroadcaster br;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener;

            // tf2::Transform accumulated_transform;
            Eigen::Affine3f accumulated_transform;
            ros::Time stamp;

            std::string camera_frame_id_, robot_base_frame_id_;

            // ros::Publisher pub_cloud;
            // RGBPointCloud::Ptr cloud;

            // Methods
            // void create_pointcloud( const cv::Mat& color, 
            //                         const cv::Mat& depth);

            void publish_all();

    }; // class CameraPoseControllerNode
} // namespace otto

#endif