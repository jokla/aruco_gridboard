#ifndef __ARUCO_GRIDBOARD_NODE_H__
#define __ARUCO_GRIDBOARD_NODE_H__

#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Header.h>
#include <opencv2/aruco.hpp>

namespace aruco_gridboard{
        class Node{
        private:
                boost::mutex lock_;
                ros::NodeHandle n_;
                unsigned long queue_size_;
                std::string board_path_;
                std::string model_description_;
                std::string detector_param_path_;
                std::string camera_frame_name_;
                bool debug_display_;
                bool status_tracker_;
                cv::Ptr<cv::aruco::Board> board_;
                image_geometry::PinholeCameraModel camera_model_;
                cv::Mat camMatrix_;
                cv::Mat_<double> distCoeffs_;


                cv_bridge::CvImagePtr cv_ptr;
                std_msgs::Header image_header_;
                bool got_image_;
                unsigned int lastHeaderSeq_;
                int freq_;

            void waitForImage();
            void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
            void publishPose();

        public:
                Node();
                ~Node();
                void spin();
        };
}
#endif
