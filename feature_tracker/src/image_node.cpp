#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>


ros::Publisher pub_match;
ros::Publisher pub_restart;


void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat tempImg = ptr->image;
    cv::Mat curImg;

    cv::resize(tempImg, curImg, cv::Size(img_msg->width/4, img_msg->height/4));
    sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(img_msg->header, "bgr8", curImg).toImageMsg();
    pub_match.publish(*msg_img);

    std::cout<<"current image size is = "<<img_msg->width/4<<", "<<img_msg->height/4<<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber sub_img = n.subscribe("/galaxy_camera/image_raw", 100, img_callback);

    pub_match = n.advertise<sensor_msgs::Image>("/galaxy_camera/image",1000);

    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?