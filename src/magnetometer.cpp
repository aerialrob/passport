
// this header incorporates all the necessary #include files and defines the class "magnetometer_class"
#include "magnetometer.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
magnetometer_class::magnetometer_class(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of magnetometer_class");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();

    // Init params
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters        
    std::vector<float> dummyParam;
    dummyParam = read_vec("magnetometer/std", 3, nh_private_);
    magnetometer_std.x = dummyParam[0];
    magnetometer_std.y = dummyParam[1];
    magnetometer_std.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/mean_normal", 3, nh_private_);
    magnetometer_mean.x = dummyParam[0];
    magnetometer_mean.y = dummyParam[1];
    magnetometer_mean.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/magnetic_vector", 3, nh_private_);
    magnetometer_vector.x = dummyParam[0];
    magnetometer_vector.y = dummyParam[1];
    magnetometer_vector.z = dummyParam[2];

    //initialize variables here, as needed
    val_to_remember_=0.0; 
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &magnetometer_class::subscriberCallback is a pointer to a member function of magnetometer_class
// "this" keyword is required, to refer to the current instance of magnetometer_class
void magnetometer_class::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &magnetometer_class::subscriberCallback,this);  
    odom_subscriber_ = nh_.subscribe("odom_in", 1, &magnetometer_class::odom_callback, this);
    magnetometer_subscriber_ = nh_.subscribe("magnetometer", 1, &magnetometer_class::magnetometer_callback, this);
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void magnetometer_class::initializeServices()
{
    ROS_INFO("Initializing Services");
    // minimal_service_ = nh_.advertiseService("exampleMinimalService",
    //                                                &magnetometer_class::serviceCallback,
    //                                                this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void magnetometer_class::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true); 
    magnetic_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("sensors/magnetometer", 1, true); 
    magnetic_from_q_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("sensors/magnetometer_from_q", 1, true);     
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}


std::vector<float> magnetometer_class::read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_)
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

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
void magnetometer_class::subscriberCallback(const std_msgs::Float32& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of magnetometer_class can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the square of the received value; 
}

void magnetometer_class::magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg) {

    // Define random generator with Gaussian distribution
    
    std::normal_distribution<double> dist_x(magnetometer_mean.x, magnetometer_std.x);
    std::normal_distribution<double> dist_y(magnetometer_mean.y, magnetometer_std.y);
    std::normal_distribution<double> dist_z(magnetometer_mean.z, magnetometer_std.z);
    magnetometer_noise.x = dist_x(generator);
    magnetometer_noise.y = dist_y(generator);
    magnetometer_noise.z = dist_z(generator);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = msg->header;
    mag_msg.magnetic_field.x = msg->magnetic_field.x + magnetometer_noise.x;
    mag_msg.magnetic_field.y = msg->magnetic_field.y + magnetometer_noise.y;
    mag_msg.magnetic_field.z = msg->magnetic_field.z + magnetometer_noise.z;
    // mag_msg.magnetic_field_covariance[0] = 0.000000080;
    // mag_msg.magnetic_field_covariance[4] = 0.000000080;
    // mag_msg.magnetic_field_covariance[8] = 0.000000080;
    magnetic_publisher_.publish(mag_msg);            
}


//member function implementation for a service callback function
// bool magnetometer_class::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
//     ROS_INFO("service callback activated");
//     response.resp = true; // boring, but valid response info
//     return true;
// }



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "magnetometer_class"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type magnetometer_class");
    magnetometer_class magnetometer_class(&nh);  //instantiate an magnetometer_class object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 

