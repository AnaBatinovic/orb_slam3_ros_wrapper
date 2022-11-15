/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
*
*/

#include "common.h"
#include <tuple>

using namespace std;

float shift = 0;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb): mpSLAM(pSLAM), mpImuGb(pImuGb){}

    void GrabImage0(const sensor_msgs::ImageConstPtr &msgRGB);
    void GrabImageDepth(const sensor_msgs::ImageConstPtr &msgD);
    std::tuple<cv::Mat, cv::Mat> GetImageRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    queue<sensor_msgs::ImageConstPtr> imgDBuf;
    std::mutex mBufMutexImage, mBufMutexDepth;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGBD_Inertial");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/robot_frame_id", robot_frame_id, "base_link");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);
  
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_raw", 100);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth_registered/image_raw", 100);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber rgb_sub = node_handler.subscribe("/camera/rgb/image_raw", 100, &ImageGrabber::GrabImage0, &igb);
    ros::Subscriber depth_sub = node_handler.subscribe("/camera/depth_registered/image_raw", 100, &ImageGrabber::GrabImageDepth, &igb);

    setup_ros_publishers(node_handler, image_transport, sensor_type);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage0(const sensor_msgs::ImageConstPtr &msgRGB)
{
    mBufMutexImage.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(msgRGB);
    mBufMutexImage.unlock();
}

void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &msgD)
{
    mBufMutexDepth.lock();
    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);
    mBufMutexDepth.unlock();
}

std::tuple<cv::Mat, cv::Mat> ImageGrabber::GetImageRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
   // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return std::make_tuple(cv_ptrRGB->image.clone(), cv_ptrD->image.clone());
}    

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat img0, imgD;
        double tIm0 = 0, tImDepth = 0;
        if (!img0Buf.empty()&&!imgDBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tIm0 = img0Buf.front()->header.stamp.toSec();
            tImDepth = imgDBuf.front()->header.stamp.toSec();

            this->mBufMutexDepth.lock();
            while((tIm0-tImDepth)>maxTimeDiff && imgDBuf.size()>1)
            {
                imgDBuf.pop();
                tImDepth = imgDBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexDepth.unlock();

            this->mBufMutexImage.lock();
            while((tImDepth-tIm0)>maxTimeDiff && img0Buf.size()>1)
            {
                img0Buf.pop();
                tIm0 = img0Buf.front()->header.stamp.toSec();
            }
            this->mBufMutexImage.unlock();

            if((tIm0-tImDepth)>maxTimeDiff || (tImDepth-tIm0)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if(tIm0>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexImage.lock();
            this->mBufMutexDepth.lock();
            auto [img0, imD] = GetImageRGBD(img0Buf.front(), imgDBuf.front());
            ros::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            imgDBuf.pop();
            this->mBufMutexImage.unlock();
            this->mBufMutexDepth.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm0)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            
          // ORB-SLAM3 runs in TrackRGBD()
          Sophus::SE3f Tcw = mpSLAM->TrackRGBD(img0, imgD, tIm0, vImuMeas);

          // Transformation from world (oriented as camera) to map frame
          Eigen::Matrix3f Tmw_matrix;
          Tmw_matrix << 1, 0, 0,
                        0, 0, 1, 
                        0, -1, 0;
          Eigen::Isometry3f Tmw(Tmw_matrix);

          // Get static transformation from camera to base link
          static tf2_ros::Buffer tfBuffer;
          static tf2_ros::TransformListener tfListener(tfBuffer);
          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform(cam_frame_id, robot_frame_id, ros::Time(0));
          }
          catch (tf2::TransformException &e) {
              ROS_ERROR("oh no cv_bridge tf exception: %s", e.what());
              return;
          }
          Eigen::Isometry3f Tcb = tf2::transformToEigen(transformStamped).cast<float>();

          // Calucate the final transformation = robot position (base link) in map frame
          Sophus::SE3f Tmb = Sophus::SE3f(Tmw.matrix()) * Tcw.inverse() * Sophus::SE3f(Tcb.matrix());
      
          publish_ros_robot_pose(Tmb, msg_time);
          publish_ros_tf_transform(Tmb, map_frame_id, robot_frame_id, msg_time);
          publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
        
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}