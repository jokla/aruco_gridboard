
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "names.h"

static const std::string OPENCV_WINDOW = "Image window";

namespace aruco_gridboard{
Node::Node() :
    n_("~"),
    queue_size_(1),
    board_path_(),
    model_description_(),
    detector_param_path_(),
    camera_frame_name_(),
    debug_display_(false),
    status_tracker_(false),
    cv_ptr(),
    image_header_(),
    got_image_(false),
    lastHeaderSeq_(0)
{
    //get the tracker configuration file
    //this file contains all of the tracker's parameters, they are not passed to ros directly.
    n_.param<std::string>("board_path", board_path_, "");
    n_.param<bool>("debug_display", debug_display_, false);
    n_.param<std::string>("detector_param_path", detector_param_path_, "");
    n_.param<std::string>("camera_frame_name", camera_frame_name_, "camera");
    n_.param("frequency", freq_, 30);
    ROS_INFO("Detector parameter file =%s",detector_param_path_.c_str());
    ROS_INFO("Board config file: =%s",board_path_.c_str());
}

Node::~Node()
{
}

void Node::waitForImage(){
    while ( ros::ok ()){
        if(got_image_) return;
        ros::spinOnce();
    }
}

static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

//records last received image
void Node::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){

    image_header_ = image->header;
    try
    {
        boost::mutex::scoped_lock(lock_);
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        camera_model_.fromCameraInfo(cam_info);

        camMatrix_ = cv::Mat(camera_model_.fullIntrinsicMatrix());
        distCoeffs_= cv::Mat(camera_model_.distortionCoeffs());
        if (distCoeffs_.size[1] < 4)
            distCoeffs_ = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    got_image_ = true;
}

//void getEulerAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles){

//  cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
//  double* _r = rotCamerMatrix.ptr<double>();
//  double projMatrix[12] = {_r[0],_r[1],_r[2],0,
//                           _r[3],_r[4],_r[5],0,
//                           _r[6],_r[7],_r[8],0};

//  cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
//                                 cameraMatrix,
//                                 rotMatrix,
//                                 transVect,
//                                 rotMatrixX,
//                                 rotMatrixY,
//                                 rotMatrixZ,
//                                 eulerAngles);
//}


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

void Node::spin(){

    //subscribe to ros topics and prepare a publisher that will publish the pose
    message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(n_, image_topic, queue_size_);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(n_, camera_info_topic, queue_size_);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
    image_info_sync.registerCallback(boost::bind(&Node::frameCallback,this, _1, _2));
    // Define Publisher
    ros::Publisher object_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
    ros::Publisher status_publisher = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);

    //wait for an image to be ready
    waitForImage();

    geometry_msgs::PoseStamped msg_pose;
    std_msgs:: Int8 status;
    ros::Rate rate(freq_);
    unsigned int cont = 0;

    // Read config file describing the board
    cv::FileStorage fs(board_path_, cv::FileStorage::READ);

    float mm_px =  fs["mm_per_unit"] ;
    mm_px *= 0.001;

    // Parse corners
    cv::FileNode corners_node = fs["corners"];
    cv::FileNodeIterator it = corners_node.begin(), it_end = corners_node.end();
    int idx = 0;
    std::vector<std::vector<float> > lbpval;
    std::vector< std::vector<cv::Point3f> > objPoints_;
    for( ; it != it_end; ++it, idx++ )
    {
        // std::cout << "Reaning corner #" << idx << ": ";
        (*it) >> lbpval;
        //std::cout << lbpval[0][0] << std::endl;
        std::vector<cv::Point3f> points;
        points.push_back(cv::Point3f(mm_px*lbpval[0][0], mm_px*lbpval[0][1], mm_px*lbpval[0][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[1][0], mm_px*lbpval[1][1], mm_px*lbpval[1][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[2][0], mm_px*lbpval[2][1], mm_px*lbpval[2][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[3][0], mm_px*lbpval[3][1], mm_px*lbpval[3][2]));
        objPoints_.push_back(points);
    }

    // Parse ids
    std::vector< int > ids_;
    fs["ids"]  >> ids_;

    fs.release();

    // Fill
    //std::vector< std::vector< cv::Point3f > > objPoints_;
    //    board_ = new cv::aruco::Board;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(1));

    //    board_->objPoints = objPoints_;
    //    board_->dictionary = dictionary;
    //    board_->ids = ids_;


    // create board object
    cv::Ptr<cv::aruco::GridBoard> gridboard =
            cv::aruco::GridBoard::create(1, 1, 0.17, 0.2, dictionary);
    cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

    //    cv::Ptr<cv::aruco::Board> board;
    //    board->create(objPoints_, dictionary, ids_);

    std::vector< std::vector<cv::Point3f> > objPoints_file = objPoints_;

    //objPoints_[0] = objPoints_[8];
    //ids_[0] = ids_[8];
    //objPoints_.resize(1);
    // ids_.resize(1);


    for (unsigned int i = 0; i< objPoints_.size(); i++)
    {
        for (unsigned int j = 0; j< objPoints_[i].size(); j++)
            std::cout<< objPoints_[i][j] << " ";
        std::cout << std::endl;
    }

    for (unsigned int i = 0; i< objPoints_.size(); i++)
    {
        std::cout<< ids_[i] << std::endl;
    }

    board->objPoints = objPoints_;
    board->dictionary = dictionary;
    board->ids = ids_;


    // CUSTOM
    //  int markersX = 3;
    //  int markersY = 3;
    //  float markerLength = 200;
    //  float markerSeparation = 50;

    //     // create board object
    //     cv::Ptr<cv::aruco::GridBoard> gridboard =
    //         cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    //     cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

    while(ros::ok()){

        if (lastHeaderSeq_ != image_header_.seq)
        {

            // Get the picture
            cv::Mat imageCopy;

            {
                boost::mutex::scoped_lock(lock_);
                cv_ptr->image.copyTo(imageCopy);
            }

            // Load parameters for the detector
            cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
            bool readOk = readDetectorParameters(detector_param_path_, detectorParams);
            if(!readOk) {
                std::cerr << "Invalid detector parameters file" << std::endl;
                return ;
            }
            detectorParams->doCornerRefinement = true; // do corner refinement in markers

            // Now detect the markers
            std::vector< int > ids;
            std::vector< std::vector< cv::Point2f > > corners, rejected;

            cv::Ptr<cv::aruco::Dictionary> dictionary  = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(0));
            cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids, detectorParams, rejected);
            std::cout << "Found  " << corners.size() << " markers." << std::endl;
            //std::cout << "Found  " << rejected.size() << " rejected." << std::endl;

            // Estimate the pose of each single markers (DEBUG)
            std::vector< cv::Vec3d > rvecs, tvecs;
            if(ids.size() > 0)
            {
                cv::aruco::estimatePoseSingleMarkers(corners, 0.17, camMatrix_, distCoeffs_, rvecs, tvecs);
                //std::cout << "r1:" <<rvecs[0] << std::endl;
                //std::cout << "t1:" <<tvecs[0] << std::endl;
                for(unsigned int i = 0; i < ids.size(); i++)
                    cv::aruco::drawAxis(imageCopy, camMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.17f * 0.5f);
            }

            //      cv::Vec3d rvecs_c, tvecs_c;

            //      for (unsigned int i = 0; i<tvecs.size(); i++)
            //      {
            //        tvecs_c += tvecs[i];
            //      }
            //      for (unsigned int i = 0; i<tvecs.size(); i++)
            //      {
            //        tvecs_c[i] = tvecs_c[i] / tvecs.size();
            //      }

            //      cv::aruco::drawAxis(imageCopy, camMatrix_, distCoeffs_, rvecs[0], tvecs_c, 0.50f * 0.5f);

            // Now estimate the pose of the board
            cv::Vec3d rvec, tvec;
            int markersOfBoardDetected = 0;

            if(ids.size() > 0)
                markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix_, distCoeffs_, rvec, tvec);

            std::cout << "size ids found:" <<ids.size() <<" " <<  board->ids.size()<< std::endl;
            std::cout << "markersOfBoardDetected:" <<markersOfBoardDetected << std::endl;
            std::cout << "objPoints:" <<corners.size() <<" " <<   board->objPoints.size()<< std::endl;
            //std::cout << "objPoints:" <<corners[0][0] <<" " <<   board->objPoints[0][0]<< std::endl;

            if(markersOfBoardDetected > 0)
            {
                status_tracker_ = 1;
                std::cout << "r:" <<rvec << std::endl;
                std::cout << "t:" <<tvec << std::endl;
                cv::aruco::drawAxis(imageCopy, camMatrix_, distCoeffs_, rvec, tvec, 0.4);
            }
            else
                status_tracker_ = 0;
            //if(rejected.size() > 0)
            //    cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

            // Draw markers on the image
            if (ids.size() > 0)
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            // DEBUG: Reproject points in the images
            //      std::vector<cv::Point2f> imagePoints;
            //      cv::projectPoints(objPoints_file[2], rvec, tvec, camMatrix_, distCoeffs_, imagePoints);
            //      //for (unsigned int i = 0; i < imagePoints.size(); ++i)
            //      //  {
            //      cv::circle(imageCopy,imagePoints[0],4,cv::Scalar(100, 0, 255),2,8,0);
            //      cv::circle(imageCopy,imagePoints[1],4,cv::Scalar(255, 0, 0),2,8,0);
            //      cv::circle(imageCopy,imagePoints[2],4,cv::Scalar(0, 0, 255),2,8,0);
            //      cv::circle(imageCopy,imagePoints[3],4,cv::Scalar(0, 255, 0),2,8,0);
            //      // }


            if (status_tracker_) {
                // Publish pose
                ros::Time now = ros::Time::now();
                msg_pose.header.stamp = now;
                msg_pose.header.frame_id = image_header_.frame_id;
                msg_pose.pose.position.x = tvec[0];
                msg_pose.pose.position.y = tvec[1];
                msg_pose.pose.position.z = tvec[2];

                cv::Mat rot_mat(3, 3, cv::DataType<float>::type);
                cv::Rodrigues(rvec, rot_mat);
                cv::Vec3d eulerAngles = rotationMatrixToEulerAngles(rot_mat);

                //std::cout << eulerAngles[2] << " " <<  eulerAngles[0] << " " << eulerAngles[1] << std::endl;

                geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
                msg_pose.pose.orientation = p_quat;

                object_pose_publisher.publish(msg_pose);

                //Publish status
                status.data = 1;
                status_publisher.publish(status);
            }
            else
            {
                status.data = 0;
                status_publisher.publish(status);
                ROS_DEBUG_THROTTLE(2, "No target detected");
            }

            cv::imshow(OPENCV_WINDOW, imageCopy);
            cv::waitKey(2);

            cont ++;

            lastHeaderSeq_ = image_header_.seq;
        }

        ros::spinOnce();
        rate.sleep();

    } //end while

    cv::destroyWindow(OPENCV_WINDOW);
} // end spin



}
