#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


// Eigen stuff
#include <eigen3/Eigen/Dense>

# define M_PI           3.14159265358979323846  /* pi */


using namespace Eigen;

class ImuQuatToDeg
{
public:
   ImuQuatToDeg(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
   : _nh(node_handle), _pnh(private_node_handle)
   {
       this->init();
   }
   ~ImuQuatToDeg() = default;

   void init();
   void cb_imu_vru2enu(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

   geometry_msgs::Vector3Stamped _orient_rpy;
   // public and private ros node handle
   ros::NodeHandle _nh;
   ros::NodeHandle _pnh;
   // Subscribers and publishers
   ros::Subscriber _sub_imu_vru;
   ros::Publisher  _pub_imu_enu;
   ros::Publisher  _pub_mag_enu;
};

void ImuQuatToDeg::init()
{
   _sub_imu_vru = _nh.subscribe("/imu_quat", 1000, &ImuQuatToDeg::cb_imu_vru2enu, this);
   _pub_imu_enu = _nh.advertise<geometry_msgs::Vector3Stamped>("/imu_rpy", 1000);
   _pub_mag_enu = _nh.advertise<sensor_msgs::MagneticField>("/mag_enu", 1000);
}


void ImuQuatToDeg::cb_imu_vru2enu(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
   // Get message
   _orient_rpy.header = msg->header;

   tf::Quaternion quat;
   double roll, pitch, yaw;
   tf::quaternionMsgToTF(msg->quaternion, quat);
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
   _orient_rpy.vector.x = roll * 180/M_PI;
   _orient_rpy.vector.y = pitch * 180/M_PI;
   _orient_rpy.vector.z = yaw * 180/M_PI;

   // Publish corrected imu message
   _pub_imu_enu.publish(_orient_rpy);

   // 1050.08 21648.5 -42827.5     // easterly , northerly, vertical (up) nanotesla
   //  msg comes in NED
   // Vector3f magnetic_field_vector{0.0000216485, 0.00000105008, 0.0000428275};   // NED TODO from param server
   Vector3f magnetic_field_vector{0.00000105008, 0.0000216485, -0.0000428275};   // ENU TODO from param server

       ////// ----------------------------------------TEST Create Magnetic Message from Orientation ------------------------
   Vector3f magnetic_field_msg;        
   // Get orientation msg
   Quaternionf qt{msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z};
   ROS_INFO_STREAM("qt " << qt.w() );
   Matrix3f R_temp;
   R_temp = qt.matrix();

   // Define random generator with Gaussian distribution
   // const double mean = 0.0;
   // const double stddev = 0.5;
   // std::default_random_engine generator;
   // std::normal_distribution<double> dist(mean, stddev);
   // Eigen::Vector3f noise{dist(generator), dist(generator), dist(generator)};     

   // Magnetic field msg 
   // magnetic_field_msg = R_temp.transpose() * magnetic_field_vector + noise; 
   magnetic_field_msg = R_temp.transpose() * magnetic_field_vector; 
   
   sensor_msgs::MagneticField mag_msg;
   mag_msg.header.frame_id = "px4";
   mag_msg.header.stamp = msg->header.stamp;
   mag_msg.magnetic_field.x = magnetic_field_msg[0];
   mag_msg.magnetic_field.y = magnetic_field_msg[1];
   mag_msg.magnetic_field.z = magnetic_field_msg[2];
   mag_msg.magnetic_field_covariance[0] = 0.000000080;
   mag_msg.magnetic_field_covariance[4] = 0.000000080;
   mag_msg.magnetic_field_covariance[8] = 0.000000080;
   _pub_mag_enu.publish(mag_msg);  
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "imu_quat_to_deg");
   ros::NodeHandle nh;
   ros::NodeHandle nh_private("~");
   ImuQuatToDeg node(nh, nh_private);
   ros::spin();

   return 0;
}