# RS1-TeamTurtleTech

install frontier exploration package

git clone https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot.git



To use the Turtle Tech Gazebo World, ensure you put the following files in the relevant directories:

Launch File (turtle_tech.launch.py):
/home/student/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch


World File (turtle_tech.world):
/home/student/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/world


Model Folders (turtle_tech, blue_guy, red_guy):
/home/student/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models


Use the following commands to create an instance of this world (Make sure to build your workspace if first time accessing this world)

If you want to make changes to the world, use the following command to be able to save it
 - XDG_CURRENT_DESKTOP=kde

Creating turtle_tech world instance
 - export TURTLEBOT3_MODEL=waffle_pi
 - ros2 launch turtlebot3_gazebo turtle_tech.launch.py
