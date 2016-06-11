#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

/* This is a visualization of what the x and y coordinates represent on 
   relative to the direction that the turtlebot is facing.
   |         X+        . (destination)
   |        |
   |        |
   |      (forward)
   |        __
   |      /   \
   |      |___|    
   |        ____________ Y-
*/


int main(int argc, char **argv)
{
  // the type of node that we have
  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  geometry_msgs::Twist cmd_vel;
  
  // We need to loop quickly for consistent movement
  ros::Rate loopRate(200); // 200 hz

  while(ros::ok())
    {
      
      // Used for debugging to see how quickly robot loops
      //robot.incrementInternalCount();
      
      // Determines which option to take;
      // Not included in nav.cpp for transparency

      switch (robot.getCurrentState()){

      case NEUTRAL:
	// We do not need to do anything
	ROS_INFO("Our yaw was %f", robot.getYaw());
	break;

      case TURN_NEG_X:
	// We need to face backwards so that numbers faces direction we move
	ROS_INFO("Rotating to face backwards.");
	if(robot.faceDestination()){
	  robot.setCurrentState(MOVE_FORWARD_X);
	}
	break;

      case MOVE_FORWARD_X:
	// We need to move forward along x-axis
	ROS_INFO("Moving forward along x-axis.");

	if(robot.goForward(X)){
	  // If y is greater than zero, we set yaw to positive for consistency
	  if(robot.getY() > 0){
	    ROS_INFO("We need to face positive y.");
	    // We need to face positive-Y axis
	    robot.setYawGoal(90);
	    robot.setCurrentState(FACE_DESTINATION);	      
	  }
	  else if(robot.getY() < 0){
	    // If yas is less than zero, we set yaw to negative for consistency
	    ROS_INFO("We need to face negative y.");
	    // We need to face negative-Y axis
	    robot.setYawGoal(-90);
	    robot.setCurrentState(FACE_DESTINATION);	      
	  }
	  else{
	    ROS_INFO("We have a y-value of 0, so we are done with movement.");
	    // We are done with movement, so just face original destination
	    robot.setCurrentState(FACE_ORIGINAL);
	    robot.setYawGoal(0);
	  }
	}
	break;

      case FACE_DESTINATION:
	// We need to face the direction of the y-coordinates
	ROS_INFO("Facing final destination.");
	if(robot.faceDestination()){
	  ROS_INFO("Done facing final destination.");
	  robot.setCurrentState(MOVE_FORWARD_Y);
	}
	break;

      case MOVE_FORWARD_Y:
	ROS_INFO("Moving forward in y axis.");

	if(robot.goForward(Y)){
	  // We need to reset the yaw to face original direction
	  // Also need to set y to 0 for positive yaw values
	  ROS_INFO("Done moving in y-axis.");
	  robot.setCurrentState(FACE_ORIGINAL);
	  robot.setY(0);
	  robot.setYawGoal(0);
	}
	break;

      case FACE_ORIGINAL:
	// Need to face original direction so that we can 
	// continue moving (weird odometry on turtlebot)
	ROS_INFO("Attempting to face original direction.");

	if(robot.faceDestination()){
	  // Done with movement, so go back to neutral
	  ROS_INFO("Done with movement.");
	  robot.setCurrentState(NEUTRAL);
	}
	break;

      default:

	ROS_INFO("Something is broken. We should not reach this point.");
      }



      loopRate.sleep();
      ros::spinOnce();
     
    }
	
  return 0;
}


