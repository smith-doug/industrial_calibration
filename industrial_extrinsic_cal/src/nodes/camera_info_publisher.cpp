#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_srvs/Empty.h>



class CameraInfoPublisher
{
private:
  std::string fixed_frame_;
  std::string scanning_frame_;

  boost::mutex lock_;
  ros::NodeHandle nh_;
  ros::Publisher camera_info_pub_;
  sensor_msgs::CameraInfo info_msg_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> PolicyType;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > depth_image_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > point_cloud_sub_;
  boost::shared_ptr<message_filters::Synchronizer<PolicyType> > data_syncronizer_;

public:
  CameraInfoPublisher(ros::NodeHandle nh);
  ~CameraInfoPublisher(){};

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& image);
  void publishInfo();
};

CameraInfoPublisher::CameraInfoPublisher(ros::NodeHandle nh)
{
  nh_ = nh;

  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  depth_image_sub_ = boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh_, "image", 1));
  point_cloud_sub_ = boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > (new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "cloud", 1));
  data_syncronizer_ = boost::shared_ptr<message_filters::Synchronizer<PolicyType> > (new message_filters::Synchronizer<PolicyType> (PolicyType(10), *point_cloud_sub_, *depth_image_sub_) );
  data_syncronizer_->registerCallback(boost::bind(&CameraInfoPublisher::callback, this, _1, _2) );

}

void CameraInfoPublisher::publishInfo()
{
  boost::lock_guard<boost::mutex> lock(lock_);
  if(info_msg_.height > 0 && info_msg_.width > 0)
  {
    info_msg_.header.stamp = ros::Time::now();
    camera_info_pub_.publish(info_msg_);
  }
}

void CameraInfoPublisher::callback(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& image)
{
  // x = (u - c_x) * d / (1000 * f_x)
  // f_x = (u - c_x) * d / (1000x)
  // y = (v - c_y) / f_y * (d / 1000)
  // f_y = (v - c_y) * d / (1000y)
  // z = d / 1000

  // iterate through point cloud and image and compute what focal length is
  // Assumptions:
  //  - The point cloud is organized and aligned with the image
  //  - The center point is the center of the image ( w/2 , h/2 )
  //  - There is no distortion

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(image, image->encoding);

  float cx = float(image->width)/2.0;
  float cy = float(image->height)/2.0;

  int count = 0;
  double tx, ty;
  pcl::PointXYZ pt;
  //std::vector<float> fx, fy;
  float fx = 0;
  float fy = 0;

  for(int j = 0; j < image->height; ++j)
  {
    for(int i = 0; i < image->width; ++i)
    {
      pt = pcl_cloud.at(i, j);

      float d = bridge->image.at<float>(j, i);

      //calculate focal length and push back
      if(std::isnan(pt.x) || pt.z == 0.0 || (pt.x < 0.1 && pt.x > -0.1) || (pt.y < 0.1 && pt.y > -0.1) || pt.z > 3.0)
      {
        continue;
      }

      //tx = (float(i) - cx) * d / (1000.0 * pt.x);
      //ty = (float(j) - cy) * d / (1000.0 * pt.y);
      tx = (float(i) - cx) * pt.z / pt.x;
      ty = (float(j) - cy) * pt.z / pt.y;

//      if(i%100 == 0)
//      {
//        ROS_INFO("i/j: %d, %d", i, j);
//        ROS_INFO("pt xyz/d: %4.3f, %4.3f, %4.3f, %5.4f", pt.x, pt.y, pt.z, d);
//        ROS_INFO("tx/ty: %4.3f, %4.3f", tx, ty);
//      }

      if (tx < 0 || ty < 0 || tx > 5000 || ty > 5000)
      {
//        ROS_WARN("bad focal length! %4.3f, %4.3f", tx, ty);
//        ROS_WARN("i/j: %d, %d", i, j);
//        ROS_WARN("pt xyz/d: %4.3f, %4.3f, %4.3f, %5.4f", pt.x, pt.y, pt.z, d);
//        ROS_WARN("tx/ty: %4.3f, %4.3f", tx, ty);
        continue;
      }

      fx += fabs(tx);
      fy += fabs(ty);
      ++count;

      //cloud->data.at(i + j*image->width)
    }
  }

  // average the fx and fy values:
  fx /= count;
  fy /= count;
  //ROS_INFO("fx/fy: %4.3f, %4.3f", fx, fy);

  double K[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  double R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double P[12] = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 0, 1};

  {
    boost::lock_guard<boost::mutex> lock(lock_);

    sensor_msgs::CameraInfo msg;
    msg.header.stamp = image->header.stamp;
    msg.height = image->height;
    msg.width = image->width;
    msg.distortion_model = "plumb_bob";
    msg.roi.width = msg.width;
    msg.roi.height = msg.height;


    for(int i = 0; i < 5; ++i)
    {
      msg.D.push_back(0);
    }

    for(int i = 0; i < 9; ++i)
    {
      msg.K[i] = K[i];
      msg.R[i] = R[i];
    }
    for(int i = 0; i < 12; ++i)
    {
      msg.P[i] = P[i];
    }
    info_msg_ = msg;
    camera_info_pub_.publish(msg);
  }


}

int main(int argc, char **argv){

  ros::init(argc, argv, "camera_info_publisher");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  CameraInfoPublisher camera_publisher(pnh);

  ros::Rate rate = ros::Rate(10);
  while(ros::ok())
  {
    camera_publisher.publishInfo();
    rate.sleep();
    ros::spinOnce();
  }


  return 0;
}
