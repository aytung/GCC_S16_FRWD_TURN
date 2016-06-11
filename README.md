## FORWARD_TURN 
ROS package that goes forward, then turns and goes forward again to reach a location determined by user

####**_Version History_**  
-------------------------  
2016-06-11: First working version
  
####**_Installing & Building_**  
-------------------------------  

Make sure ROS is properly installed on your machine by follow these instructions:

http://gcc-robotics.github.io/robot-motion-planning-labs/lab0-setting-up/

**A. Create Catkin workspace in your home directory:**  

``` 
mkdir -p ~/catkin_ws/src  
```  

**B. Initialize the workspace:**  

```  
cd ~/catkin_ws/src  
catkin_init_workspace  
cd ~/catkin_ws  
catkin_make  
```  
  
**C. Load the proper environment:**  

``` 
source devel/setup.bash  
```  

**D. Verify that your workspace is in ROS's path:**  

``` 
echo $ROS_PACKAGE_PATH  
```  
  *YOU SHOULD SEE: /home/YOUR_USER_NAME/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks*  


**E. Clone the code from GitHub:**  

``` 
cd ~/catkin_ws/src  
git clone https://github.com/aytung/GCC_S16_TRN_FRWD.git
``` 

**F. Build the code:**
```    
cd ~/catkin_ws  
catkin_make  
``` 

####**_Running The Package**  
-------------------------------  

There are two Nodes included:  

| Nodes                    | Functions                                               |
| ------------------------ |---------------------------------------------------------|
| ui_node                  | simple publisher that send two double variables         |
| nav_node                 | subscriber take takes responds to vars sent by ui_node  |
  
  
**A. Open 3 shell windows**  

**B. In all 3 shells, load the environment**

```
source ~/catkin_ws/devel/setup.bash
```

**C. In the first window, run launch turtlebot_bringup**  

```
roslaunch turtlebot_bringup minimal.launch
```

**D. In the second and third windows, run these pair of nodes**

*For simple pubsub with two std_msg types*
```
rosrun turtlebot nav_node
rosrun turtlebot ui_node
```

**E. Enter appropriate coordinate values**


*Enter coordinates between 9 and -9 in the window where you ran the command "rosrun turtlebot ui_node"
```
Enter your coordinates.
X: 1.4
Y: -1.2
Coordinates sent.

```
