#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class ElektronTeleopJoy {
public:
	ElektronTeleopJoy();
	void publish();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

        geometry_msgs::Twist vel;

};

ElektronTeleopJoy::ElektronTeleopJoy() {
	nh_.param("axis_linear", linear_, 1);
	nh_.param("axis_angular", angular_, 0);
	nh_.param("scale_angular", a_scale_, 1.0);
	nh_.param("scale_linear", l_scale_, 0.23);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy> ("joy", 10,
			&ElektronTeleopJoy::joyCallback, this);

}

void ElektronTeleopJoy::publish() {
        vel_pub_.publish(vel);
}

void ElektronTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	vel.angular.z = a_scale_ * joy->axes[angular_];
	vel.linear.x = l_scale_ * joy->axes[linear_];
<<<<<<< HEAD
    //vel_pub_.publish(vel);
=======
//    vel_pub_.publish(vel);
>>>>>>> 463660c1f3568aa9b7bc8f3c6d0eb6f6b58e2411
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_turtle");
	ElektronTeleopJoy elektron_teleop;

	ros::Rate loop_rate(50);

  	while(ros::ok())
	{
<<<<<<< HEAD
        elektron_teleop.publish();
=======
		elektron_teleop.publish();
>>>>>>> 463660c1f3568aa9b7bc8f3c6d0eb6f6b58e2411

		ros::spinOnce();
		loop_rate.sleep();
	}

}
