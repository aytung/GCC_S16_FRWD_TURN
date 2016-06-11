#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "std_msgs/Float64.h"
#include <turtlebot/mymsg.h>
#include <sstream>

// This file accepts integer values between 9 and -9, and
// translates those values into movement 
int main(int argc, char **argv)
{
  double x = 0;
  double y = 0;

  ros::init(argc, argv, "talker2");
  
  // Node that will be handling ui
  ros::NodeHandle n;

  // "turtlebot" indicates name of package;
  // "mymsg" indicates the name of the message file
  // '"my_msg"' indicates the name of the topic we are publishing to
  ros::Publisher chatter1_pub = n.advertise<turtlebot::mymsg>("my_msg", 1000);

  // Doesn't have to loop quickly, since we aren't going to send messages
  // very quickly
  ros::Rate loop_rate(1);


  while (ros::ok())
  {

    turtlebot::mymsg msg;

    x = 0;
    y = 0;
    std::cout << "Enter your coordinates." << std::endl;
    std::cout << "X:";
    std::cin >> x;
    msg.x=x;

    std::cout << "Y:";
    std::cin >> y;
    msg.y=y;
    

    // When coordinates are within acceptable bounds
    if( abs(x) <= 9 && abs(y) <= 9){
      chatter1_pub.publish(msg);
      std::cout << "Coordinates sent." << std::endl;
      }
    // We cannot send coordinates over 9 and less than -9
    // since the classroom is smaller than that
      else{
	std::cout << "Error! Your coordinates must both be within a range of -9 and 9 (inclusive)." << std::endl;
	std::cout << "Coordinates not sent." << std::endl;
      }

    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
