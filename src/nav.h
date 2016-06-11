#ifndef NAV_H
#define NAV_H
// ROS includes.
#include "ros/ros.h"
#include <turtlebot/mymsg.h>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "create_node/TurtlebotSensorState.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>


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

using namespace std;

enum State{ NEUTRAL, TURN_NEG_X, MOVE_FORWARD_X,FACE_DESTINATION, MOVE_FORWARD_Y, FACE_ORIGINAL };
enum Direction{X, Y};    

class RoboState
  {
  public:

    // constructor
    RoboState(ros::NodeHandle rosNode);
    double getYaw();

    // These are public so that we can use main for clarity
    void setY(double y);
    double getY();
    State getCurrentState();
    void incrementInternalCount();
    bool faceDestination();
    int getInternalCount();
    bool goForward(Direction currentDirection);
    void setYawGoal(double newYawGoal);
    void setCurrentState(State newState);
  private:

    // the ros node being used by RoboState
    ros::NodeHandle node;    

    // publishers and subscribers 
    geometry_msgs::Twist velocityCommand;
    ros::Publisher velocityPublisher;
    geometry_msgs::Twist command;
    ros::Subscriber messageSubscriber;
    ros::Subscriber bumperSubscriber;

    State currentState;
    // various callback functions
    void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
    void messageCallback(const turtlebot::mymsg::ConstPtr& msg);
    void setMessageStatus(bool status);    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    ros::Subscriber odomSubscriber;
    int count;
    int internalCount;
    void determineYawGoal();

    // private variables
    double xOdom;
    double yOdom;
    double xCoord;
    double yCoord;
    double acceptErr;
    double yawErr;
    double yaw;
    double yawGoal;

    // get and set functions

    void setXodom(double xOdom);
    void setYaw(double newYaw);
    void setYodom(double yOdom);
    double getYawGoal();
    void setErr(double err);
    double getErr();
    double getXodom();
    double getYodom();
    double getXodomOld();
    double getYodomOld();
    void setXodomOld(double xOdomCurrent);
    void setYodomOld(double yOdomCurrent);
    double getX();
    void setX(double x);

  };

#include "nav.cpp"
#endif // NAV_NODE_H
