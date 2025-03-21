# 1.Create ROS Workspace
mkdir -p voice_controlturtfinal_ws/src
cd voice_controlturtfinal_ws
catkin_make

# 2.Create ROS Package
cd src
catkin_create_pkg voice_robot rospy std_msgs geometry_msgs
# Install python libraries for voice recognition
sudo apt-get install python3-pip python3-pyaudio
pip3 install SpeechRecognition

# 3.Launch simulation world
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# 4.Create Nodes
1.speaker_node.py
2.control_node.py

# 5.Make nodes executable
chmod +x voice_controlturtfinal_ws/src/voice_robot/scripts/speaker_node.py
chmod +x voice_controlturtfinal_ws/src/voice_robot/scripts/control_node.py

# 6.Run speaker node
cd voice_controlturtfinal_ws
source devel/setup.bash
rosrun voice_control speaker_node.py
Also run control node

# 7.Test Voice command now and control turtlebot.
