#! /usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from vitarana_drone.srv import Gripper
from vitarana_drone.srv import GripperRequest
from vitarana_drone.srv import GripperResponse

from gazebo_logical_camera.msg import LogicalCameraImage


class Gripper():

    # Constructor
    def __init__(self):
        
        param_config_gripper = rospy.get_param('config_gripper')
        
        self._gripper_model_name = param_config_gripper['gripper_model_name']
        self._gripper_link_name = param_config_gripper['gripper_link_name']
        
        self._object_model_name = param_config_gripper['attachable_object_model_name']
        self._object_link_name = param_config_gripper['attachable_object_link_name']
        
       
        self._logical_camera_topic_name = param_config_gripper['logical_camera_topic_name']
        
        print(param_config_gripper)
        
        self._flag_pickable = False

        

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self._attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        self._attach_srv_a.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self._attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        self._attach_srv_d.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        rospy.loginfo( '\033[94m' + " >>> Gripper init done." + '\033[0m')

    
    def activate_gripper(self):
      
      rospy.loginfo("Attach request received")
      req = AttachRequest()
      req.model_name_1 = self._gripper_model_name
      req.link_name_1 = self._gripper_link_name
      req.model_name_2 = self._object_model_name
      req.link_name_2 = self._object_link_name
      self._attach_srv_a.call(req)

    
    
    def deactivate_gripper(self):
      
      rospy.loginfo("Detach request received")
      req = AttachRequest()
      req.model_name_1 = self._gripper_model_name
      req.link_name_1 = self._gripper_link_name
      req.model_name_2 = self._object_model_name
      req.link_name_2 = self._object_link_name
      self._attach_srv_d.call(req)

    
    def callback_service_on_request(self, req):
        rospy.loginfo( '\033[94m' + " >>> Gripper Activate: {}".format(req.activatem_gripper) + '\033[0m')
        rospy.loginfo( '\033[94m' + " >>> Gripper Flag Pickable: {}".format(self._flag_pickable) + '\033[0m')

        if( (req.activate_gripper == True) and (self._flag_pickable == True) ):
            self.activate_gripper()
            return GripperResponse(True)
        else:
            # self._flag_pickable = False
            self.deactivate_gripper()
            return GripperResponse(False)
        
        

    def callback_topic_subscription(self, rx_msg):
        # rospy.loginfo( '\033[94m' + "{}".format(rx_msg) + '\033[0m')
        
        number_models = len(rx_msg.models)
        
        flag_attachable_object_found = False

        for i in range(0, number_models):
            name_model = rx_msg.models[i].type
            
            if(name_model == self._attachable_object_prefix):
                rospy.loginfo( '\033[94m' + " >>> Gripper: Pickable object found {}".format(name_model) + '\033[0m')
                
                self._object_model_name = name_model

                flag_attachable_object_found = True
                self._flag_pickable = True
                break
            
        if(flag_attachable_object_found == False):
            self._flag_pickable = False
            
        
        

    # Destructor
    def __del__(self):
        rospy.loginfo( '\033[94m' + " >>> Gripper Del." + '\033[0m')





def main():
    rospy.init_node('node_service_server_gripper')
    
    eDrone_gripper = Gripper()
    
    s = rospy.Service('/eyrc/vb/eDrone_1/activate_gripper', Gripper, eDrone_gripper.callback_service_on_request)
    rospy.loginfo( '\033[94m' + " >>> Gripper Activation Service Ready." + '\033[0m')
    
    rospy.Subscriber(eDrone_gripper._logical_camera_topic_name, LogicalCameraImage, eDrone_gripper.callback_topic_subscription)
    
    rospy.spin()


if __name__ == "__main__":
    main()