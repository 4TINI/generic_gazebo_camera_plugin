#!/usr/bin/env python3

import os
import roslaunch
from roslaunch.parent import ROSLaunchParent
import rospkg
import rospy

class ROSLauncher(object):
    def __init__(self, rospackage_name, launch_path, arguments=None):
        self._rospackage_name = rospackage_name
        self.rospack = rospkg.RosPack()
        self._path_launch_file_name = launch_path

        # Check Package Exists
        try:
            pkg_path = self.rospack.get_path(rospackage_name)
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package NOT FOUND...")
    
        # If the package was found then we launch
        if pkg_path:
            if(arguments is not None):
                roslaunch_file = [(self._path_launch_file_name, arguments)]    
            else:
                roslaunch_file = [self._path_launch_file_name]                                      
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = ROSLaunchParent(self.uuid, 
                                         roslaunch_file)        
        else:
            assert False, ("No Package Path was found for ROS package ==>" + 
                          str(self._rospackage_name))

    def start(self):
        self.launch.start()
        
    def restart(self, arguments=None):
        if self._path_launch_file_name == None:
            assert False, ("No Package Path was found for ROS package ==>" + 
                          str(self._rospackage_name))
        else:
            self.launch.shutdown()
            if(arguments is not None):
                roslaunch_file = [(self._path_launch_file_name, arguments)]    
            else:
                roslaunch_file = [self._path_launch_file_name] 
            #a double check before starting launch file again
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = ROSLaunchParent(self.uuid, 
                                         roslaunch_file)
            self.launch.start()
    
    def shutdown(self):
        self.launch.shutdown()
