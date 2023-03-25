#include <ros/ros.h>

#include <node_camera_pose_controller.h>

#ifdef DVO_CHRONO
    #include <chrono>
    #include <iomanip>
    using Time = std::chrono::high_resolution_clock;
    using double_sec = std::chrono::duration<double>;
    using time_point = std::chrono::time_point<Time, double_sec>;
#endif

namespace otto {

    CameraPoseControllerNode::CameraPoseControllerNode():
        found_camera_to_baselink_(false),
        sync(SyncPolicy(10), color_sub, depth_sub),
        tf_listener(tf_buffer),
        rate(15),
        camera_info_set_(false),
        dvo(3, false, true, 20.0f, 100, 1e-5f)
    {
        std::string nodename = ros::this_node::getName();
        ros::NodeHandle nh("~");

        nh_ = nh;

        std::string color_topic, depth_topic, camera_intrinsics_file;
        nh_.param<std::string>(nodename + "/color_topic", color_topic, nodename + "/raw/color_image");
        nh_.param<std::string>(nodename + "/depth_topic", depth_topic, nodename + "/raw/depth_image");
        nh_.param<std::string>(nodename + "/frame_id", camera_frame_id_, "camera");

        color_sub.subscribe(nh_, color_topic, 1);
        depth_sub.subscribe(nh_, depth_topic, 1);

        bool robot_to_base_available = false;
        if (nh_.getParam(nodename + "/robot_base_frame_id", robot_base_frame_id_)) {
            ROS_INFO("Transform from base link to camera frame provided");
            robot_to_base_available = true;
        }

        // Load camera intrinsics from file
        if (!nh_.getParam(nodename + "/camera_intrinsics_file", camera_intrinsics_file)) {
            throw std::runtime_error("'camera_intrinsics_file' mandatory parameter was not defined.");
        }
        dvo.update_camera_info(camera_intrinsics_file);

        sync.registerCallback(&CameraPoseControllerNode::callback, this);

        int levels = 3;
        bool use_gpu = false;
        bool use_weighter = true;
        float sigma = 20.0f;
        int max_iterations = 100;
        float tolerance = 1e-5f;

        // pointcloud = nh_.param<bool>("pointcloud", false);

        // if(pointcloud)
        // {
        //     ROS_INFO("PointCloud creation enabled");

        //     cloud.reset(new RGBPointCloud);
        //     cloud->header.frame_id = "/camera_link";
        //     cloud->is_dense = false;
        //     cloud->height = 240;
        //     cloud->width = 424;
        //     cloud->points.resize(cloud->height*cloud->width);

        //     pub_cloud = nh_.advertise<RGBPointCloud>(   ros::this_node::getNamespace()      \
        //                                                 + "/" + ros::this_node::getName()   \
        //                                                 + "/cloud", 5);
        // }

        // At start we assume robot's at map's origin
        if (robot_to_base_available) {
            geometry_msgs::TransformStamped tf_msg;
            Eigen::Affine3d base_to_camera;
            try
            {
                float timeout = 3.0f;
                ROS_INFO(
                    "Looking for transform from '%s' to '%s' for '%f' seconds",
                    robot_base_frame_id_.c_str(), camera_frame_id_.c_str(), timeout
                );
                tf_msg = tf_buffer.lookupTransform(
                    camera_frame_id_, robot_base_frame_id_, ros::Time(0), ros::Duration(timeout)
                );
                base_to_camera = tf2::transformToEigen(tf_msg);
                accumulated_transform = base_to_camera.cast<float>();
            
            } catch(tf2::TransformException& e) {
                ROS_ERROR("%s", e.what()); 
                accumulated_transform.setIdentity();           
            }
        } else {
            accumulated_transform.setIdentity();
        }

        ROS_INFO("Camera Controller Node initialized");
    }

    CameraPoseControllerNode::~CameraPoseControllerNode()
    { 
    }

    void CameraPoseControllerNode::callback(
        const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
    {
        cv_bridge::CvImagePtr color_ptr, depth_ptr;
        try
        {
            color_ptr = cv_bridge::toCvCopy(color, sensor_msgs::image_encodings::TYPE_8UC3);
            depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s", e.what());
            return;
        }

        #ifdef DVO_CHRONO
            time_point start = Time::now();
        #endif

        Eigen::Affine3f estimated_transform;
        estimated_transform.matrix() = dvo.step(
            color_ptr->image, depth_ptr->image, vo::util::Mat4f::Identity()
        );
        ROS_INFO("here");
        #ifdef DVO_CHRONO
            time_point end = Time::now();
            ROS_INFO("Elapsed time: %d", (end - start).count());
        #endif
        
        // Update base_link pose
        accumulated_transform = accumulated_transform * estimated_transform.inverse();

        stamp = color->header.stamp;

        // // Update pointcloud if required
        // if(pointcloud)
        //     create_pointcloud(color_ptr->image, depth_ptr->image);
        
    }

    // void CameraPoseControllerNode::create_pointcloud( const cv::Mat& color, 
    //                                     const cv::Mat& depth)
    // {

    //     Mat3f K = dvo.get_camera_matrix();
    //     float scale = dvo.get_scale();
    //     float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);

    //     int idx = 0;

    //     const float bad_point = std::numeric_limits<float>::quiet_NaN();

    //     #pragma omp parallel for schedule(dynamic) num_threads(std::thread::hardware_concurrency())
    //     for(int y = 0; y < depth.rows; y++)
    //     {
    //         const cv::Vec3b* rgb_ptr;
    //         const ushort* depth_ptr;

    //         #pragma omp critical
    //         {
    //             rgb_ptr = color.ptr<cv::Vec3b>(y);
    //             depth_ptr = depth.ptr<ushort>(y);
    //         }

    //         for(int x = 0; x < depth.cols; x++)
    //         {
    //             pcl::PointXYZRGB p;                
    //             float z = ((float) depth_ptr[x]) * scale;

    //             if ( z > 0.0f)
    //             {
    //                 p.z = z;
    //                 p.x = p.z*(x - cx)/fx;
    //                 p.y = p.z*(y - cy)/fy;

    //                 int b = (int) rgb_ptr[x][0];
    //                 int g = (int) rgb_ptr[x][1];
    //                 int r = (int) rgb_ptr[x][2];
    //                 int rgb = (r << 16) + (g << 8) + b;
    //                 p.rgb = (float) rgb;
    //             }
    //             else
    //             {
    //                 p.x = p.y = p.z = p.rgb = bad_point;
    //             }

    //             #pragma omp critical
    //             cloud->points[idx] = p;

    //             idx++;
    //         }
    //     }
    // }

    void CameraPoseControllerNode::publish_all()
    {
        // // Publish pointcloud if required
        // if(pointcloud && (pub_cloud.getNumSubscribers() > 0))
        //     pub_cloud.publish(*cloud);

        // Broadcast robot's pose
        Eigen::Vector3f t = accumulated_transform.translation();
        Eigen::Quaternionf q(accumulated_transform.rotation());
        q.normalize();

        geometry_msgs::TransformStamped tf_msg_out;
        tf_msg_out.header.stamp = stamp;
        tf_msg_out.header.frame_id = "map";
        tf_msg_out.child_frame_id = robot_base_frame_id_;
        tf_msg_out.transform.translation.x = t.x();
        tf_msg_out.transform.translation.y = t.y();
        tf_msg_out.transform.translation.z = t.z();
        tf_msg_out.transform.rotation.x = q.x();
        tf_msg_out.transform.rotation.y = q.y();
        tf_msg_out.transform.rotation.z = q.z();
        tf_msg_out.transform.rotation.w = q.w();

        br.sendTransform(tf_msg_out);
    }

    void CameraPoseControllerNode::run()
    {
        while(ros::ok())
        {
            try{
                ros::spinOnce();
                publish_all();
                rate.sleep();
            }
            catch(std::runtime_error& e)
            {
                ROS_ERROR("%s", e.what());
            }
        }
    }

} // namespace otto

int main(int argc, char** argv)
{
    // query_devices();
    ros::init(argc, argv, "camera_pose_controller");
    otto::CameraPoseControllerNode node;

    // ros::spin();
    node.run();

    return 0;
}