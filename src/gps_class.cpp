//gps_class.cpp:
//wsn, Feb 2015
//illustrates how to use classes to make ROS nodes
// constructor can do the initialization work, including setting up subscribers, publishers and services
// can use member variables to pass data from subscribers to other member functions

// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun gps_class gps_class
// rostopic echo exampleMinimalPubTopic
// rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0
// rosservice call exampleMinimalService 1


// this header incorporates all the necessary #include files and defines the class "gpsRosClass"
#include "gps_class.h"



using namespace GeographicLib;


//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
gpsRosClass::gpsRosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of gpsRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    val_to_remember_=0.0; 

    // Init params
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters        
    std::vector<float> dummyParam;
    dummyParam = read_vec("gps/origin", 3, nh_private_);
    gps_origin.x = dummyParam[0];
    gps_origin.y = dummyParam[1];
    gps_origin.z = dummyParam[2];
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

std::vector<float> gpsRosClass::read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_)
{
    std::vector<double> params(exp_long);    

    XmlRpc::XmlRpcValue my_list;
    nh_private_.getParam(param_name, my_list);

    if (!nh_private_.hasParam(param_name))    
        std::cout << "ERROR loading " << param_name << ": Not declared" << std::endl;
    else
        ROS_INFO_STREAM("Loaded from parameter server  " << param_name);

    if (my_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        int vec_size = int(my_list.size());
        if (vec_size == exp_long)
        {
            for (int ii = 0; ii < vec_size; ii++)
            {
                params[ii] = my_list[ii];     
                std::cout << params[ii] << std::endl;
            }
                         
        }
        else
            std::cout << "ERROR loading " << param_name << ": Wrong elements number" << std::endl;
    }
    else
        std::cout << "ERROR loading " << param_name << ": Wrong element type" << std::endl;
    
    std::vector<float> paramsf(params.begin(), params.end());
    return paramsf;
}


//member helper function to set up subscribers;
// note odd syntax: &gpsRosClass::subscriberCallback is a pointer to a member function of gpsRosClass
// "this" keyword is required, to refer to the current instance of gpsRosClass
void gpsRosClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");    
    gps_subscriber_ = nh_.subscribe("gps_in", 1, &gpsRosClass::gps_callback,this);
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void gpsRosClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    // add more services here, as needed
}

//member helper function to set up publishers;
void gpsRosClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");    
    gps_publisher_ = nh_.advertise<nav_msgs::Odometry>("/local_pose", 1, true);
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

void gpsRosClass::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{        
    if (!isfirst_gps)
        isfirst_gps = true;

    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    const double lat0 = gps_origin.x, lon0 = gps_origin.y , h0 = gps_origin.z; // 
    LocalCartesian proj(lat0, lon0, h0, earth);
    
    // Sample forward calculation
    double lat = msg->latitude, lon = msg->longitude, h = msg->altitude; 
    double x, y, z;
    proj.Forward(lat, lon, h, x, y, z);
    ROS_INFO("latitude [%f] longitude [%f] altitude [%f]", lat, lon, h);
    ROS_INFO("x [%f] y [%f] z [%f]", x, y, z);      

    // nav_msgs::Odometry odom_msg_;
    // odom_msg_.header.seq = this->seq_;
    odom_msg_.header.stamp = msg->header.stamp;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = z;

    gps_publisher_.publish(odom_msg_);


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x,y,z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
}


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "gpsRosClass"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type gpsRosClass");
    gpsRosClass gpsRosClass(&nh);  //instantiate an gpsRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 

