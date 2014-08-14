#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// wheel diameter in SI units [m]
#define WHEEL_DIAM 0.1
// regulator rate in SI units [Hz]
#define REGULATOR_RATE 100
// number of encoder ticks per single wheel rotation
#define ENC_TICKS 5000

#define MAX_PWM 2000 

#define KEYCODE_SPC 0x20

class badanie {
private:
	geometry_msgs::Twist vel_;
	ros::NodeHandle nh_;
	double l_scale_, a_scale_,procent;
	ros::Publisher vel_pub_;
public:
	void init() {
		nh_.param("scale_angular", a_scale_, 1.0);
                nh_.param("scale_linear", l_scale_, 0.23);
                vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

                ros::NodeHandle n_private("~");
        }

        void keyboardLoop();


};
int kfd = 0;
struct termios cooked, raw;
double procent;
void quit(int sig) {
        tcsetattr(kfd, TCSANOW, &cooked);
        exit(0);
}

int main(int argc, char** argv) {

        ros::init(argc, argv, "elektron_teleop_keyboard");
        puts("moving robot set parameter to percent of PWM");
        puts("------set-PWM-value---------------------");
;
        badanie tpk;
        tpk.init();

        signal(SIGINT, quit);

        tpk.keyboardLoop();

        return (0);
}
void badanie::keyboardLoop() {
        char c;
        bool dirty = false;

        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

      //  puts("moving robot set parameter to percent of PWM");
       // puts("---------------------------");

        std::cin>>procent;

	vel_.linear.x = 0;
	vel_.angular.z=0;
    double vel = (procent/100.0)*MAX_PWM;
	std::cout<<vel<<std::endl;
	for(;;) {

               // dirty = false;
                // get the next event from the keyboard
       /*         if (read(kfd, &c, 1) < 0) {
                        perror("read():");
                        exit(-1);
                }
	*/	
                vel_.linear.x=vel*(2 * 3.14 * WHEEL_DIAM * REGULATOR_RATE)/ENC_TICKS;
	//	std::cout<<vel_.linear.x<<std::endl;
          /*      switch (c) {
                
                case KEYCODE_SPC:
                        vel_.linear.x = 0;
                        vel_.angular.z = 0;
                        dirty = true;
                //      std::cout << "SPACE"<<std::endl;
                        break;
                }

 */
	        vel_pub_.publish(vel_);



        }
}



