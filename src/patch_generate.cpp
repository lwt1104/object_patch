#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <baxter_core_msgs/EndpointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/RecognizedObject.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include <stdio.h>

class DetectedObject {
  public:
  cv::Vec3d point;
  std::string key;
  cv::Scalar color;
  DetectedObject(cv::Vec3d p, std::string k, cv::Scalar c):point(p), key(k), color(c){
  }

};

std::map<std::string, int> key2color;
cv::Scalar colormap[3] = {cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255)};
static const std::string OPENCV_WINDOW = "Image window";
std::vector<DetectedObject> detect_objs;
bool pointReady = false;
bool shouldCrop = false;
int image_nb = 0;
std::string prefix;



void imageCallback(const sensor_msgs::Image& msgs_image) {

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Draw an example circle on the video stream
  if (!pointReady) {
    return;
  }
  pointReady = false;
  std::vector<DetectedObject> detect_objs_copy = detect_objs;
  cv::Mat image_copy;
  if (shouldCrop) {
    image_copy = cv_ptr->image.clone(); 
  }
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
    int offset = 40;
    for (std::size_t i = 0; i < detect_objs.size(); i++) {
      if (shouldCrop) {
        cv::Rect myROI(detect_objs[i].point[0] - offset, detect_objs[i].point[1] - offset, 2 * offset + 10, 2 * offset + 10);
        cv::Rect imageArea(0, 0, cv_ptr->image.cols, cv_ptr->image.rows);

        cv::Mat croppedImage = image_copy(myROI);
        std::stringstream ss;
        ss << "./images/"<< prefix << image_nb++ << ".jpg";
        imwrite( ss.str(), croppedImage );
      }
      
      // ROS_INFO("u:%f  v:%f %f key: %s", detect_objs[i].key.c_str(), pointUV[1], pointUV[2],detect_obj.key.c_str());
      std::cout << "  " << detect_objs[i].color << "  "<< detect_objs[i].key << std::endl;
      cv::rectangle(cv_ptr->image, cv::Point(detect_objs[i].point[0] - offset, detect_objs[i].point[1] - offset), cv::Point(detect_objs[i].point[0] + offset + 10, detect_objs[i].point[1] + offset + 10), detect_objs[i].color, 3);
      
    }  
    
  }
  detect_objs.clear();
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(1);
    

}

void callback(const object_recognition_msgs::RecognizedObjectArray& obj_array)
{
 
  std_msgs::Header h = obj_array.header;
  const std::vector<object_recognition_msgs::RecognizedObject> objects = obj_array.objects;
  // ROS_INFO ("Data");
  ROS_INFO ("%d", objects.size());
  cv::Matx33d K (525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0);
  
  
  for (std::size_t i = 0; i < objects.size(); i++) {
      double px = objects[i].pose.pose.pose.position.x;
      double py = objects[i].pose.pose.pose.position.y;
      double pz = objects[i].pose.pose.pose.position.z;
      // filter out points we don't care
      ROS_INFO("depth:%f", pz);
      if (pz < 1.7) {

        cv::Vec3d point(px / pz, py / pz, 1.0);
        cv::Vec3d pointUV = K * point;
        
        if (key2color.find(objects[i].type.key) == key2color.end()) {
          key2color[objects[i].type.key] = key2color.size();
        }
        DetectedObject detect_obj(pointUV, objects[i].type.key, colormap[key2color[objects[i].type.key]]); 
        detect_objs.push_back(detect_obj);
        ROS_INFO("u:%f  v:%f %f key: %s", pointUV[0], pointUV[1], pointUV[2],detect_obj.key.c_str());
        if (i == objects.size() - 1) {
          pointReady = true;
        }
      }
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listen_recognized_object_array");
  cv::namedWindow(OPENCV_WINDOW);

  if (argc > 1) {
    shouldCrop = true;
    prefix = std::string(argv[1]);
  }
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/recognized_object_array", 10, callback);
  ros::Subscriber subCoord = nh.subscribe("/camera/rgb/image_color", 1, imageCallback);
  ros::spin();
}