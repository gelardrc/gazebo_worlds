#!/usr/bin/env python

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState,GetModelState
from geometry_msgs.msg import Pose,Point,Quaternion,Wrench,Vector3
from gazebo_msgs.msg import ModelState

class world():
    
    def __init__(self):
        
        self.init_clients()

        
        pass
    def init_clients(self):
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        
        rospy.wait_for_service('/gazebo/get_model_state')
        
        rospy.wait_for_service('/gazebo/set_model_state')
        
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    
        pass
    
    def spwan(self,pose,model_path,model_name):
        
        p = Point(pose[0],pose[1],pose[2])
        o = Quaternion(0,0,0,0)
        pose = Pose(position= p,orientation=o)
        self.spawn_model(
	        	model_name=model_name,
	        	model_xml=open(model_path, 'r').read(),
	        	robot_namespace='/foo',
	        	initial_pose= pose,
	        	reference_frame='world'
	    	)
        pass
    
    def move_model(self,model_name,vel):
        

        t = self.get_state(model_name,"world")

        new_pose = Pose()
        new_pose.position.x = t.pose.position.x + vel[0]
        new_pose.position.y = t.pose.position.y + vel[1]
        new_pose.position.z = t.pose.position.z + vel[2]
        new_pose.orientation = t.pose.orientation
        self.set_state(ModelState(
            model_name = model_name,
            pose = new_pose
        ))


        
        


        pass

def constroi_mundo():
    mundo = world()
    
    rospack = rospkg.RosPack()
    

    dir = rospack.get_path('gazebo_worlds')
    

    obj_i = 10
    obj_j = 10
    
    
    for i in range(obj_i):
        for j in range(obj_j):
            mundo.spwan(pose =[7*i+7,6*j+6,1],model_path =dir+'/models/new_solar/model.sdf' ,model_name = 'panel_'+str(i)+'_'+str(j))

    

    rospy.loginfo("All models were spawn.")
if __name__ == '__main__':
    
    rospy.init_node('world',anonymous=False)
    
    constroi_mundo()