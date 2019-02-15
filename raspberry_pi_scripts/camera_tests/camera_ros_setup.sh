sudo apt-get install ros-kinetic-compressed-image-transport
sudo apt-get install ros-kinetic-image-common
sudo apt-get install ros-kinetic-image-view
cd ~/catkin_ws/src
git clone https://github.com/dganbold/raspicam_node.git
cd ~/catkin_ws/
catkin_make --pkg raspicam_node
cd ~/catkin_ws/
source devel/setup.bash
roslaunch raspicam_node camera_module_v2_640x480_30fps.launch
# View Image
rosservice call /raspicam_node/start_capture
# View Strem
rosrun image_view image_view image:=/raspicam_node/image_raw
rosrun rqt_image_view rqt_image_view
