roslaunch racecar_gazebo racecar_tunnel_genie.launch & roslaunch racecar_navigation slam.launch localization:=true database_path:=~/tunnel_genie.db & roslaunch racecar_navigation rviz.launch & roslaunch racecar_behaviors behaviors.launch & roslaunch racecar_behaviors blob_detection.launch


