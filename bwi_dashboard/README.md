bwi_dashboard
=================
- Created by Asher Johnson
- Email: IAmAsherJohnson@gmail.com

Feel free to contact me with any questions.

This tool is an experimental web dashboard to control various components of a robot running ROS. The main use of this tool was planned around the BWI project, yet it can be used by any robot capable of running the dependencies listed below.

##Dependencies

There are 3 main components that must be run in order for the application to work as intended (Note: Not all of the dependencies are required to run the dashboard, but only the tools listed in "Used in" will work):
- rosbridge
 - Used to communicate topics to and from the dashboard
 - Used in: Teleop, Navigation, Sensors
 - Run using: `roslaunch rosbridge_server rosbridge_websocket.launch`
- mjpeg_server
 - Used to stream video to the dashboard
 - Used in: Video
 - Run using: `rosrun mjpeg_server mjpeg_server __image_transport:=compressed`
- robot_pose_publisher
 - Used to stream location of robot
 - Used in: Navigation
 - Run using: `rosrun robot_pose_publisher robot_pose_publisher`

##Widgets

- Message Log
 - Shows output from javascript log and topics provided in js/config.js by the variable logTopics.
- Video
 - Displays the output of video topics.
 - Video quality can be chosen from a value between 0-100.
 - Multiple topics can be subscribed to and can be changed via the edit button on the video that is displayed.
- Teleop
 - Controls the robot's velocity.
 - The speed scale can be set between 0-100.
 - The buttons can be used to control the robot.
 - The QWEASD keys can also be used on a laptop or desktop computer.
- Navigation
 - Shows the current map loaded onto the robot.
 - If robot_pose_publisher is running, it will display the robot's position (will display as a yellow arrow).
 - The screen can be double clicked to place a navigation goal (will display as a pink arrow).
 - The initial location of the map is based on the 0,0 coordinate location of the map.
 - The arrow buttons and zoom buttons can be used to relocate the map to a usable location.
- Sensors
 - Shows the JSON output of subscribed topics.
 - Subtopics can be provided to simplify the output provided, as the whole path to that element. For example, both "twist/twist/linear" and "twist/twist" are valid subtopics of "/odom".

##Created Files

There are several files required to run this code, but the following files are the only ones that I created:

- index.html : The main file that includes both the configuration screen and the dashboard screen
- js/config.js : The configuration file that includes necessary variables and configuration defaults
- js/CustomKeyboardTeleop.js : It is the keyboard teleop plugin created as part of the Robot Web Tools project, but I had to customize it to allow the buttons to control the commands

##How To Use

There is a file, `js/config.js`, that contains default configurations that can be used with this program. However, you can also input connection and topic information on the configuration screen of the dashboard.

Open the index.html page in a browser or host it on your web server and access the location of the index.html file. First input the proper configuration information or select pre-configured configurations from the drop down lists, then click connect. When the dashboard displays, open the panels that you need. Note: for Teleop, the Q,W,E,A,S,D keys can be used for control as well.

There are option url parameters to control the choice of default configuration. "c" controls the connection configuration and "s" controls the subscription configuration. The choice of configuration is case sensitive. An example of this url would be http://farnsworth.csres.utexas.edu/bwi_experimental/bwi_dashboard/index.html?c=hypnotoad&s=driving
