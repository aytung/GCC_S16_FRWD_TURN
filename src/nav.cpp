#ifndef NAV_CPP
#define NAV_CPP
#include <turtlebot/mymsg.h>
#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// the least amount the turtlebot can move, otherwise odometry breaks;
// serves as our acceptable error
const double ANGLE_ERR = 7;
// The velocity that we publish along angular.z to rotate
const double ROTATION_VELOCITY = .7;
// The velocity that we publish along linear.x to move forward
const double FORWARD_VELOCITY = 0.2;

// how much we need to multiply radians by in order to get degrees
const double ANGLE_CONVERT = 57.2958;



// initializer for RoboState::State (with default values)
// We initialize coordinates and odometry to zero, since robot starts at origin;
// We set the first state to NEUTRAL and acceptable error to .03
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), yaw(0), yawGoal(0), currentState(NEUTRAL), internalCount(0), acceptErr(.03)
{
  // declare which ROS node we use
  this->node = rosNode;
  // publishes velocity messages
  this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  // subscribes to messages from ui_node
  this->messageSubscriber= this->node.subscribe("my_msg", 1000, &RoboState::messageCallback, this);
  // subscribes to data from bumpers
  this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 100, &RoboState::bumperCallback, this);
  // subscribes to encoders
  this->odomSubscriber = this->node.subscribe("/odom", 100, &RoboState::odomCallback, this);
}

// @return Whether we are done with forward movement
// face the final destination, which is determined by yawGoal
bool RoboState::faceDestination()
{
  bool done = false;
  ROS_INFO("Calling face destination.");

  double yawOffset = getYawGoal() - getYaw();

  // Tell us how far off we were
  ROS_INFO("yawOffset was %f", yawOffset);
  
  // We were not within acceptable error
  if(yawOffset <= -ANGLE_ERR || yawOffset >= ANGLE_ERR){
    this->velocityCommand.linear.x = 0.0;
    // We need to rotate left
    if(yawOffset > 0){
      this->velocityCommand.angular.z = ROTATION_VELOCITY;
    }
    // We need to rotate right
    else{
      this->velocityCommand.angular.z = -ROTATION_VELOCITY;
    }
    
  }
  else {
    // We are done with rotation
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = 0.0;
    done = true;
  }
  velocityPublisher.publish(this->velocityCommand);
  // For debugging
  //  ROS_INFO("Yaw is %f", getYaw());
  return done;
}


// @args The current direction we are facing; either X or Y
// @return Whether or not we have finished movement in the direction indicates by args
// Goes forward along whichever axis and returns true or false to tell us if
// we are done with movement along that axis
bool RoboState::goForward(Direction currentDirection)
{
  // By default, assume we are not done
  bool done = false;
  
  ROS_INFO("Yaw is %f", getYaw());
  
  // We are facing the X direction
  if (currentDirection == X){
    // How much we are off by in the x-direction
    double xOffset = getX() - getXodom();
    
    // Offset was not within acceptable bounds
    if(xOffset <= -getErr() || xOffset >= getErr()){
      ROS_INFO("Moving forward because we are off by %f.", xOffset);
      this->velocityCommand.linear.x = FORWARD_VELOCITY;
    }
    // Offset was within acceptable bounds
    else{
      ROS_INFO("Done with forward movement in the X direction.");
      this->velocityCommand.linear.x = 0.0;
      done = true;
    }
  }
  // We are facing the Y direction
  else{
    double yOffset = getY() - getYodom();
    
    // Offset was not within acceptable bounds
    if(yOffset <= -getErr() || yOffset >= getErr()){
  this->velocityCommand.linear.x = FORWARD_VELOCITY;
    }
    // Offset was within acceptable bounds
    else{
      ROS_INFO("Done with forward movement in the Y direction.");
      this->velocityCommand.linear.x = 0.0;
      done = true;
    }
  }

  // We are never rotating in this case
  this->velocityCommand.angular.z = 0.0;    
  velocityPublisher.publish(this->velocityCommand);

  return done;
}

// The function that we use whenever we receive odometry data from turtlebot
void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  // set odometry
  setXodom(msg->pose.pose.position.x);
  setYodom(msg->pose.pose.position.y);
  
  // convert from quaternions to angles
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, tempYaw;
  m.getRPY(roll, pitch, tempYaw);
  
  tempYaw*=ANGLE_CONVERT;
  // determine whether to send positive or negative angles
  bool isTempYawNeg = (tempYaw < 0);
  
  // we always want a positive yaw value  when Y >= 0(for consistency)
  if(getY() >= 0){
    if(isTempYawNeg){
      setYaw(360+tempYaw);
    }
    else{
      setYaw(tempYaw);
    }
  }
  // we always want a negative yaw value when y is negative(for consistency)
  else{
    if(isTempYawNeg){
      setYaw(tempYaw);
    }
    else{
      setYaw(-360+tempYaw);
    }
  }

  // we do not want to get spammed with messages about yaw.
  if(getInternalCount() % 100==1){
    ROS_INFO("The yaw was %f", getYaw());

    if(getY() >= 0){
      ROS_INFO("The yaw is supposed to be positive.");
    }
    else{
      ROS_INFO("The yaw is supposed to be negative.");
    }
  }
}

// The function that is called whenever we receive a message
void RoboState::messageCallback(const turtlebot::mymsg::ConstPtr& msg)
{
  // only accept message if movement is not in progress
  if(getCurrentState()==NEUTRAL)
    {
      if(msg->x==0 && msg->y==0)
	ROS_INFO("No reason to move a distance of 0. Message not sent.");
      else{	  
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);

	// We need to odometry and message to account for prior movement
	setX(msg->x + getXodom());
	setY(msg->y + getYodom());

	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	
	// we don't need to face backward since initial movement is forward
	if(getX() > 0){
	  setCurrentState(MOVE_FORWARD_X);
	}
	// need to face backward since initial movement is backward
	// (want bumper sensors to be useful)
       	else if (getX() < 0){
	  if(getY() >= 0){
	    setYawGoal(180);
	  }
	  else{
	    setYawGoal(-180);
	  }
	  setCurrentState(TURN_NEG_X);
	}
	else{
	  setCurrentState(FACE_DESTINATION);
	  setErr(.05);
      }
      
      }
    }
  else{
    ROS_INFO("Cannot accept message. Movement still in progress.");
  }
}

// Make robot stop if we receive bumper feedback
void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
  // if bumpers don't complain, don't run the loop
  if(msg->bumps_wheeldrops != 0){
    ROS_INFO("You hit an object! Motion terminating.");
    ROS_INFO("The remaining x was:%f and the remaining y was: %f.", getX(), getY());
    setX(0);
    setY(0);
    // allow RoboState::State to receive messages again
    setCurrentState(NEUTRAL);
  }
}

/*
  Section for dealing with internalCount

*/


void RoboState::incrementInternalCount()
{
  internalCount++;
}

int RoboState::getInternalCount()
{
  return internalCount;
}

/* 

   Get and set functions

*/

void RoboState::setCurrentState(State newState)
{
  currentState = newState;
}

State RoboState::getCurrentState()
{
  return currentState;
}

double RoboState::getX()
{
  return xCoord;
}

void RoboState::setX(double x)
{
  xCoord=x;
}

void RoboState::setY(double y)
{
  yCoord=y;
}

double RoboState::getY()
{
  return yCoord;
}

double RoboState::getXodom()
{
  return xOdom;
}

double RoboState::getYodom()
{
  return yOdom;
}

void RoboState::setXodom(double xOdomCurrent)
{
  xOdom = xOdomCurrent;
}

void RoboState::setYodom(double yOdomCurrent)
{
  yOdom = yOdomCurrent;
}

double RoboState::getErr()
{
  return acceptErr;
}

void RoboState::setErr(double err)
{
  acceptErr = err;
}

void RoboState::setYaw(double newYaw)
{
  yaw = newYaw;
}

double RoboState::getYaw()
{
  return yaw;
}

void RoboState::setYawGoal(double newYawGoal)
{
  yawGoal = newYawGoal;
}

double RoboState::getYawGoal()
{
  return yawGoal;
}
 
#endif
