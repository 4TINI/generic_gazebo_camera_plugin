#!/usr/bin/env python3

import rospy
import yaml
import json
import utils
import numpy as np

class ConfigParser(object):
    def __init__(self, cameras_config_file):
        
        self.cameras_config_file = cameras_config_file
        
        try:
            with open(self.cameras_config_file, "r") as stream:
                self.data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr(exc)
        
        s_num = ''
        self.cameras_numbers = []
        self.cameras_brands = []
        self.cameras_models = []
        self.cameras_serial_numbers = []
        self.cameras_topics_prefixes = []
        self.cameras_params = []
        
        for cnt, camera_name in enumerate(self.data['setup']):
            for ch in str(camera_name):
                if ch.isdigit():
                    s_num = ch
                    self.cameras_numbers.append(int(s_num))
            # camera_brand = str(self.data['setup'][camera_name]['camera_brand'])
            # camera_model = str(self.data['setup'][camera_name]['camera_model'])  
            # camera_serial_number = str(self.data['setup'][camera_name]['serial_no'])  
            
            # self.cameras_brands.append(camera_brand)   
            # self.cameras_models.append(camera_model)  
            # self.cameras_serial_numbers.append(camera_serial_number)                      

            self.cameras_topics_prefixes.append('/camera' + s_num)
            params = []
            for param_cnt, param_value in enumerate(self.data['setup'][camera_name]):
                params.append(str(param_value)+':='+str(self.data['setup'][camera_name][param_value]))
            
            self.cameras_params.append(params)
            
    def get_info_topics(self):
        self.cameras_info_topics_list = [sub + '/rgb/camera_info' for sub in self.cameras_topics_prefixes]
        return self.cameras_info_topics_list
    
    def get_color_topics(self):     
        self.cameras_color_topics_list = [sub + '/rgb/image_raw' for sub in self.cameras_topics_prefixes]
        return self.cameras_color_topics_list
    
    def get_openpose_topics(self):
        self.cameras_openpose_topics_list = [sub + '/aligned_depth_to_color/image_raw' for sub in self.cameras_topics_prefixes]
        return self.cameras_openpose_topics_list
    
    def get_calibration_optical_frames(self):
        self.calibration_optical_frames = [sub + '_color_optical_frame' for sub in self.cameras_topics_prefixes]
        self.calibration_optical_frames = [e[1:] for e in self.calibration_optical_frames]
        return self.calibration_optical_frames
    
    def get_serial_numbers(self):
        return self.cameras_serial_numbers
    
    def get_base_links(self):
        self.base_links = [sub + '_link' for sub in self.cameras_topics_prefixes]
        self.base_links = [e[1:] for e in self.base_links]
        return self.base_links
    
    def get_cameras_numbers(self):
        return self.cameras_numbers
    
    def get_cameras_topic_prefixes(self):
        return self.cameras_topics_prefixes

    def get_cameras_brands(self):
        return self.cameras_brands
    
    def get_cameras_models(self):
        return self.cameras_models
    
    def get_cameras_params(self):
        return self.cameras_params
    
    def get_data(self):
        return self.data
    
class ExtrinsicParser(object):
    def __init__(self, cameras_extrinsic_calibration_otuput_file):
        
        self.cameras_extrinsic_calibration_otuput_file = cameras_extrinsic_calibration_otuput_file
        self.transforms = []
        self.matrices = []
        try:
            with open(self.cameras_extrinsic_calibration_otuput_file,"r") as file:
                self.data = json.load(file)
        except json.JSONDecodeError as exc:
            rospy.logerr(exc)
        
        for i in self.data['transforms_to_root_cam']:
            tf = np.reshape(i['transform']['data'], (i['transform']['rows'], i['transform']['cols']))  
            self.matrices.append(tf)
            self.transforms.append(utils.fromTransfMatrixToTransformStamped(tf))
            
        
    def get_extrinsic_tf(self):
        return self.transforms
    
    def get_extrinsic_matrices(self):
        return self.matrices