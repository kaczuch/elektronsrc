#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <pthread.h>
#include "serialcomm/serialcomm.hpp"
#include "nf/nfv2.h"
#include "math.h"

#define TIMEOUT 100

// wheel diameter in SI units [m]
#define WHEEL_DIAM 0.1

// axle length in SI units [m]
#define AXLE_LENGTH 0.335

// regulator rate in SI units [Hz]
#define REGULATOR_RATE 100

// maximum velocity, in internal units
#define MAX_VEL 100000

// number of encoder ticks per single wheel rotation
#define ENC_TICKS 5000

ros::Time cmd_time;

NF_STRUCT_ComBuf	NFComBuf;
SerialComm		*CommPort;
uint8_t txBuf[256];	//dane wysylane do plytki
uint8_t txCnt;		//dlugosc danych wysylanych do plytki
uint8_t rxBuf[256];	//dane odebrane do plytki
uint8_t rxCnt;		//dlugosc danych odebranych od plytki
uint8_t commandArray[256];	//tablica polecen do sterownika
uint8_t commandCnt;		//stopien zapelnienia tablicy polecen do sterownika
uint8_t rxCommandArray[256];	
uint8_t rxCommandCnt;		
int timeoutCount = 0;
int left = 0;


bool wyjscie=false;
void readDeviceVitalsTimerCallback(const ros::TimerEvent&) 
{
	commandArray[commandCnt++] = NF_COMMAND_ReadDeviceVitals;
	commandArray[commandCnt++] = NF_COMMAND_ReadDrivesPosition;
}

void twistCallback(const geometry_msgs::TwistConstPtr& msg) 
{
	//wyliczenie proporcji skretu do ruchu postepowego
	double rotational_term = msg->angular.z * AXLE_LENGTH / 2.0;

	//wyliczenie predkosci kol	
    double rvel = ((msg->linear.x + rotational_term) * ENC_TICKS) / (2 * 3.14 * WHEEL_DIAM * REGULATOR_RATE);
    double lvel = ((msg->linear.x - rotational_term) * ENC_TICKS) / (2 * 3.14 * WHEEL_DIAM * REGULATOR_RATE);

    //double rvel = msg->linear.x;
    //double lvel = msg->linear.x;
    // czy skalowanie w ten sposob nie wpłynie na charakterystyke ruchu?

	if (rvel > MAX_VEL)

        rvel = MAX_VEL;
	else if (rvel < -MAX_VEL)
		rvel = -MAX_VEL;

	if (lvel > MAX_VEL)
		lvel = MAX_VEL;
	else if (lvel < -MAX_VEL)
		lvel = -MAX_VEL;
		
	NFComBuf.SetDrivesSpeed.data[0] = lvel;
	NFComBuf.SetDrivesSpeed.data[1] = rvel;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesSpeed;
	if(left != lvel)
	{
		left = lvel;
		ROS_INFO("Added SetDrivesSpeed Command %d, %d", NFComBuf.SetDrivesSpeed.data[0], NFComBuf.SetDrivesSpeed.data[1]);
		
	}
	
	NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
	NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
	//ROS_INFO("Added SetDrivesMode Command NF_DrivesMode_SPEED, NF_DrivesMode_SPEED");
}

void *listener(void *p)
{
	while(1) 
	{
		if(rxCnt == 255)
			rxCnt = 0;
		rxBuf[rxCnt] = CommPort->readOneByte();
		//if(CommPort->read(&(rxBuf[rxCnt]), 1) > 0)
		//{
			//ROS_INFO("RECEIVED %x", rxBuf[rxCnt]);
			if(NF_Interpreter(&NFComBuf, rxBuf, &rxCnt, rxCommandArray, &rxCommandCnt) > 0)
			{
				if(NFComBuf.ReadDrivesPosition.updated){
					ROS_INFO("Wheel speed: %d, %d", NFComBuf.ReadDrivesPosition.data[0], NFComBuf.ReadDrivesPosition.data[1]);


				NFComBuf.ReadDrivesPosition.updated=0;
				}
				//ROS_INFO("Message Received!");
			}
		//}
	}
	pthread_exit(NULL);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "elektron_base_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	bool publish_odom_tf;
	bool dump;

	double ls, rs;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry> ("odom", 1);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber twist_sub = n.subscribe("cmd_vel", 1, &twistCallback);
	ros::Rate loop_rate(200);
	std::string dev;

	if (!nh.getParam("device", dev)) 
	{
		dev = "/dev/ttyACM0";
	}
	if (!nh.getParam("publish_odom_tf", publish_odom_tf)) 
	{
		publish_odom_tf = false;
	}
	if (!nh.getParam("dump", dump)) 
	{
		dump = false;
	}
	if (!nh.getParam("lin_scale", ls)) 
	{
                ls = 1.0;
        }
	if (!nh.getParam("rot_scale", rs)) 
	{
                rs = 1.0;
        }

	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	
	// NFv2 Configuration
	NFv2_Config(&NFComBuf, NF_TerminalAddress);
    
        CommPort = new SerialComm(dev);

	if (! (CommPort->isConnected()) ) 
	{
		ROS_INFO("connection failed");
		ROS_ERROR("Connection to device %s failed", dev.c_str());
		return 0;
	}
	else 
	{
		ROS_INFO("connection ok");
		
		pthread_t listenerThread;
		if(pthread_create(&listenerThread, NULL, &listener, NULL))
			ROS_ERROR("Listener thread error");
		else
			ROS_INFO("Listener thread started");
			
		ros::Timer timer1 = n.createTimer(ros::Duration(0.1), readDeviceVitalsTimerCallback);
		
		while (ros::ok()) 
		{
			double x, y, th, xvel, thvel;

			ros::Time current_time = ros::Time::now();
			
			// If communication with Elektron requested
			if(commandCnt > 0) 
			{
				txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf, (const uint8_t*)commandArray, commandCnt, NF_RobotAddress);
				// Clear communication request
				commandCnt = 0;
				// Send command frame to Elektron
				CommPort->write(txBuf, txCnt);
				timeoutCount = 0;
			}/*else
			{
				if(timeoutCount < TIMEOUT)
				{
					txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf, (const uint8_t*)commandArray, commandCnt, NF_RobotAddress);
					CommPort->write(txBuf, txCnt);
					commandCnt = 0;					
					timeoutCount++;
				}
				else
				{
					NFComBuf.SetDrivesSpeed.data[0] = 0;
				        NFComBuf.SetDrivesSpeed.data[1] = 0;
				        commandArray[commandCnt++] = NF_COMMAND_SetDrivesSpeed;

 				        NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
       					NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
				        commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
					txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf, (const uint8_t*)commandArray, commandCnt, NF_RobotAddress);
                                        CommPort->write(txBuf, txCnt);
					commandCnt = 0;
				}
			}
*/
			x = 0;
			y = 0;
			th = 0;
			xvel = 0;
			thvel = 0;

			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			if (publish_odom_tf) 
			{
				//first, we'll publish the transform over tf
				odom_trans.header.stamp = current_time;

				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;

				//send the transform
				odom_broadcaster.sendTransform(odom_trans);
			}

			//next, we'll publish the odometry message over ROS
			odom.header.stamp = current_time;

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			odom.pose.covariance[0] = 0.00001;
			odom.pose.covariance[7] = 0.00001;
			odom.pose.covariance[14] = 10.0;
			odom.pose.covariance[21] = 1.0;
			odom.pose.covariance[28] = 1.0;
			odom.pose.covariance[35] = thvel + 0.001;

			//set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = xvel;
			odom.twist.twist.linear.y = 0.0;
			odom.twist.twist.angular.z = thvel;

			//publish the message
			//odom_pub.publish(odom);

			ros::spinOnce();
			loop_rate.sleep();
		}
	} 
	return 0;
}
