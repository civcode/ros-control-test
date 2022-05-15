#include "ros_control_test/robot_hardware_interface.h"

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 100;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	motor_effort_pub = nh_.advertise<std_msgs::Float32>("/motor_effort", 10);

    encoder_speed_sub = nh_.subscribe("/encoder_speed", 10, &ROBOTHardwareInterface::encoderSpeedCb, this);
    encoder_position_sub = nh_.subscribe("/encoder_position", 10, &ROBOTHardwareInterface::encoderPositionCb, this);
    velocity_controller_command_sub = nh_.subscribe("/single_joint_actuator/joint1_velocity_controller/command", 10, &ROBOTHardwareInterface::velocityControllerCb, this);
	

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    
	joint_name_="joint1";
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_, &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandle);

    //jointStateHandle.

// Create position joint interface
    //hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
    //position_joint_interface_.registerHandle(jointPositionHandle);
    
// Create velocity joint interface
	//hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    //velocity_joint_interface_.registerHandle(jointVelocityHandle);
    //effort_joint_interface_.registerHandle(jointVelocityHandle);
    

// Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_);
	effort_joint_interface_.registerHandle(jointEffortHandle);

    //jointEffortHandle.getCommandPtr()
    //effort_joint_interface_.getHandle(joint_name_);
	
// Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits("joint1", nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle);
	
/*
If you have more joints then,

    joint_name_= "joint2"
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle2(joint_name_, &joint_position_2, &joint_velocity_2, &joint_effort_2);
    joint_state_interface_.registerHandle(jointStateHandle);

//create the position/velocity/effort interface according to your actuator 
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_2);
    position_joint_interface_.registerHandle(jointPositionHandle2);
    
    hardware_interface::JointHandle jointVelocityHandle2(jointStateHandle2, &joint_velocity_command_2);
    effort_joint_interface_.registerHandle(jointVelocityHandle2);
    
    hardware_interface::JointHandle jointEffortHandle2(jointStateHandle2, &joint_effort_command_2);
	effort_joint_interface_.registerHandle(jointEffortHandle2);
	
//create joint limit interface.
    joint_limits_interface::getJointLimits("joint2", nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle2(jointEffortHandle2, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle2);
	
	Repeat same for other joints
*/
	

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    //registerInterface(&velocity_joint_interface_);
    //registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
}


void ROBOTHardwareInterface::encoderSpeedCb(const std_msgs::Float32 &msg) {

    encoder_speed_ = msg.data;
    
}

void ROBOTHardwareInterface::encoderPositionCb(const std_msgs::Float32 &msg) {

    encoder_position_ = msg.data;
    //encoder_position_ = 0.0f;
    
}

void ROBOTHardwareInterface::velocityControllerCb(const std_msgs::Float64 &msg) {

    velocity_command_ = static_cast<float>(msg.data);
    //encoder_position_ = 0.0f;
    
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

    joint_velocity_ = static_cast<double>(encoder_speed_);
    joint_position_ = static_cast<double>(encoder_position_);
    static int cnt;
    //if (cnt++%100 == 0)
	//    ROS_INFO("encoder: %.2f %.2f", joint_velocity_, joint_position_);

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    //effortJointSaturationInterface.enforceLimits(elapsed_time);    
	//joints_pub.data.clear();
	//joints_pub.data.push_back(joint_effort_command_);
    //joint_state_interface_.getHandle()
	
    
    //jointEffortHandle.getCommandPtr()
    //hardware_interface::JointHandle handle = effort_joint_interface_.getHandle(joint_name_);
    //double command =  *(handle.getCommandPtr());
	//ROS_INFO("command [joint_effort_cmd_] = %.2f", command);

	//ROS_INFO("command [] = %.2f", velocity_command_);

    //hardware_interface::JointHandle v_handle = velocity_joint_interface_.getHandle(joint_name_); 
    //v_handle.

    //controller_interface::ControllerBase *base = controller_manager_->getControllerByName("joint1_velocity_controller");
    //base->state_();
    
	
    effortJointSaturationInterface.enforceLimits(elapsed_time);    

	//ROS_INFO("PWM Cmd: %.2f", joint_effort_command_);
	//pub.publish(joints_pub);
    std_msgs::Float32 msg;
    msg.data = static_cast<float>(joint_effort_command_);
    //y = 0.06689466*x + 53.47114
    float forward_feed = 0.06689466 * velocity_command_ + 53.47114;
    msg.data += forward_feed;
    //msg.data += 50;
    motor_effort_pub.publish(msg);
	//ROS_INFO("effort [ff, cont, sum] = [%.2f, %.2f, %.2f]", forward_feed, joint_effort_command_, forward_feed+joint_effort_command_);
		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(3); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
