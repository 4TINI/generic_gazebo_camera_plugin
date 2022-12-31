#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg
from sensor_msgs.msg import CameraInfo

from utils.cameras_config_file_utils import ConfigParser
from utils.launch_manager import ROSLauncher

if __name__ == '__main__':

    rospy.init_node('spawn_cameras')
    
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    pack_path = rospack.get_path('generic_gazebo_camera_plugin')
    
    # read params
    config_file = rospy.get_param('~cameras_config_file', pack_path + '/config/default.yaml')
    
    # Wait for ROS time
    wait_rate = rospy.Rate(1.0)
    while(not rospy.Time.now()):
        wait_rate.sleep()
        
    config_dict = ConfigParser(config_file)
    cameras_info_topics_list = config_dict.get_info_topics()
    
    cameras_launch_list = []

    for count, value in enumerate(config_dict.get_cameras_numbers()):
        launch_args =[]
        launch_args.append(pack_path+'/launch/spawn_camera.xml')
        launch_args.append('camera_number:='+str(value))
        launch_args.append('launch_gazebo:=False')
        
        launch_args.extend(config_dict.get_cameras_params()[count]) 

        spawn_camera_launch = ROSLauncher('generic_gazebo_camera_plugin',
                                        rospack.get_path('generic_gazebo_camera_plugin')+'/launch/spawn_camera.xml',
                                        launch_args)
                                        
        spawn_camera_launch.start()

        rospy.loginfo("Waiting for topic: %s to be published", '/camera'+str(value)+'/rgb/camera_info')
        rospy.wait_for_message('/camera'+str(value)+'/rgb/camera_info', CameraInfo)

    while not rospy.is_shutdown():        
        rospy.spin()