#!/usr/bin/python


import roslib; roslib.load_manifest('cob_script_server')
import threading
from copy import deepcopy

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface



class simple_moveit_interface:


    _transform_listener = None
    _transform_listener_creation_lock = threading.Lock()

    _mgc_dict = dict()
    _mgc_dict_creation_lock = threading.Lock()

    _psi = None
    _psi_creation_lock = threading.Lock()


    def __init__(self):
        pass

    def MoveLin(self, target_frame = "arm_ee_link", goal_frame = "base_link", x, y, z, r, p, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = goal_frame
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        [rx, ry, rz, rw] = tf.transformations.quaternion_from_euler(r, p, y, 'sxyz')

        goal_pose.pose.orientation.x = rx
        goal_pose.pose.orientation.y = ry
        goal_pose.pose.orientation.z = rz
        goal_pose.pose.orientation.w = rw

        self.MoveLin(target_frame, goal_pose)

    def MoveLin(self, target_frame = "arm_ee_link", goal_pose):
        self.pubPose.publish(goal_pose)
        mgc = self.get_move_group_commander("arm")
        print "Set endeffector to " , target_frame
        mgc.set_end_effector_link(target_frame)
        print "Received MoveLin target pose: ", goal_pose
        control_goal = self.convertToBaseLink(goal_pose)
        print "Control Goal:", control_goal
        mgc.set_pose_reference_frame("base_link")
        path,fraction = mgc.compute_cartesian_path([control_goal.pose], 0.02, 3, True)
        print "Fraction: ", fraction
        if fraction > 0.95:
            mgc.execute(path)
            return True
        else:
            return False

    def MovePTP(self, target_frame = "arm_ee_link", goal_pose):
        mgc = self.get_move_group_commander("arm")
        print "Set endeffector to " , target_frame
        mgc.set_end_effector_link(target_frame)
        print "Received MovePTP target pose: ", goal_pose
        control_goal = self.convertToBaseLink(goal_pose)
        print control_goal
        mgc.set_pose_target(control_goal, target_frame)
        print "Go now"
        if mgc.go() == True:
            return True
        else:
            return False
        


    def moveit_joint_goal(group, goal, replanning=False):
        mgc = get_move_group_commander(group)
        mgc.allow_replanning(replanning)
        if mgc.go(goal):
            print "Done moving"
            return 'succeeded'
        else:
            print "Failed to plan path"
            return 'failed'

    def moveit_pose_goal(group, ref_frame, goal, replan=False):
        mgc = get_move_group_commander(group)
        mgc.allow_replanning(replan)
        mgc.set_pose_reference_frame(ref_frame)
        ret = mgc.go(goal)
        mgc.allow_replanning(False)
        if ret:
            print "Done moving"
            return 'succeeded'
        else:
            print "Failed to plan path"
            return 'failed'
    
    

    #this is linear movement
    def moveit_cart_goals(group, ref_frame, goal_list, avoid_collisions=True):
        mgc = get_move_group_commander(group)
        
        mgc.set_pose_reference_frame(ref_frame)
        (traj,frac)  = mgc.compute_cartesian_path(goal_list, 0.01, 4, avoid_collisions)
        print traj,frac

        #mgc.execute(traj)
        #print "Done moving"
        #return 'succeeded'  
        
        if frac == 1.0:
            if mgc.execute(traj):
            print "Done moving"
                return 'succeeded'
        else:
            print "Something happened during execution"
            print 'failed'
        else:
            print "Failed to plan full path!"
            return 'failed'


    def moveit_get_current_pose(group):
        mgc = get_move_group_commander(group)

        cp = mgc.get_current_pose()
        cp.header.stamp = rospy.Time()
        return cp



    def attach_mesh_to_link(self, link, name, path):
        psi = self.get_planning_scene_interface()
        pose = PoseStamped()
        pose.pose.orientation.w = 1
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = link
        psi.attach_mesh(link, name, pose, path)


    ####################################
    #  helpers

    def convertToBaseLink(self, goal_pose, target="base_link"):
        if(goal_pose.header.frame_id != target):
              
            goal_in_bl = deepcopy(goal_pose)

            self.transformer.waitForTransform(target, goal_pose.header.frame_id, rospy.Time(), rospy.Duration(8.0))
              
            print "Converting from ", goal_pose.header.frame_id, " to ", target
            now = rospy.Time.now()
            goal_pose.header.stamp = now

            rospy.sleep(0.1);

            self.transformer.waitForTransform(target, goal_pose.header.frame_id, now, 
            rospy.Duration(8.0))

            print "wait done"

            #goal.goal_pose.header.stamp = self.transformer.getLatestCommonTime("/base_link", goal.goal_pose.header.frame_id)

            (trans,rot) = self.transformer.lookupTransform(target, goal_pose.header.frame_id, now)
            print "tansform: ", trans
            print "rot: ", rot

            print "doing the actual tranform"
            try:
            goal_in_bl = self.transformer.transformPose(target, goal_pose)
                except Exception as e:
            print "Error during transformPose"
            print e

            print "Transformed MoveLin target to: ", goal_in_bl
            for i in range(0,5):
                self.broadcaster.sendTransform([goal_in_bl.pose.position.x, goal_in_bl.pose.position.y, goal_in_bl.pose.position.z] , [goal_in_bl.pose.orientation.x, goal_in_bl.pose.orientation.y, goal_in_bl.pose.orientation.z, goal_in_bl.pose.orientation.w] , goal_in_bl.header.stamp, "MovePTPFrame", target)
                self.pubs.publish(goal_in_bl)
            return goal_in_bl
        else:
            print "Conversion not required"
            return goal_pose

    def add_ground(self):
        psi = self.get_planning_scene_interface()
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
        pose.pose.position.z = -0.0501
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "base_link"
        psi.attach_box("base_link", "ground", pose, (3, 3, 0.1))

    def clear_objects(self):
        psi = self.get_planning_scene_interface()
        psi.remove_attached_object("arm_7_link")
        psi.remove_world_object("")
        
    def clear_attached_object(self, object_name):
        psi = self.get_planning_scene_interface()
        psi.remove_attached_object(link="arm_7_link", name = object_name)
        psi.remove_world_object(object_name)

    def get_transform_listener(self):
        '''
        Gets the transform listener for this process.

        This is needed because tf only allows one transform listener per process. Threadsafe, so
        that two threads could call this at the same time, at it will do the right thing.
        '''
        self._transform_listener
        with self._transform_listener_creation_lock:
            if self._transform_listener == None:
                self._transform_listener = tf.TransformListener()
            return self._transform_listener



    def get_move_group_commander(self, group):
        '''
        Gets the move_group_commander for this process.

        '''
        with self._mgc_dict_creation_lock:
            if not group in self._mgc_dict:
                self._mgc_group = MoveGroupCommander(group)
                self._mgc_group.set_planner_id('RRTConnectkConfigDefault')
                self._mgc_dict[group] = self._mgc_group
            self.add_ground()
            return self._mgc_dict[group]



    def get_planning_scene_interface(self):
        '''
        Gets the planning_scene_interface for this process.

        '''
        with self._psi_creation_lock:
            if self._psi == None:
                self._psi = PlanningSceneInterface()
            return self._psi


    def get_goal_from_server(self, group, parameter_name):

            ns_global_prefix = "/script_server"

            # get joint_names from parameter server
            param_string = ns_global_prefix + "/" + group + "/joint_names"
            if not rospy.has_param(param_string):
                    rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
                    return None
            joint_names = rospy.get_param(param_string)
            
            # check joint_names parameter
            if not type(joint_names) is list: # check list
                    rospy.logerr("no valid joint_names for %s: not a list, aborting...",group)
                    print "joint_names are:",joint_names
                    return None
            else:
                for i in joint_names:
                    if not type(i) is str: # check string
                        rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",group)
                        print "joint_names are:",joint_names
                        return None
                    else:
                        rospy.logdebug("accepted joint_names for group %s",group)
            
            # get joint values from parameter server
            if type(parameter_name) is str:
                if not rospy.has_param(ns_global_prefix + "/" + group + "/" + parameter_name):
                    rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/" + group + "/" + parameter_name)
                    return None
                param = rospy.get_param(ns_global_prefix + "/" + group + "/" + parameter_name)
            else:
                param = parameter_name

            # check trajectory parameters
            if not type(param) is list: # check outer list
                    rospy.logerr("no valid parameter for %s: not a list, aborting...",group)
                    print "parameter is:",param
                    return None


            #no need for trajectories anymore, since planning (will) guarantee collision-free motion!
            point = param[len(param)-1]

            #print point,"type1 = ", type(point)
            if type(point) is str:
                if not rospy.has_param(ns_global_prefix + "/" + group + "/" + point):
                    rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/" + group + "/" + point)
                    return None
                point = rospy.get_param(ns_global_prefix + "/" + group + "/" + point)
                point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
                #print point
            elif type(point) is list:
                rospy.logdebug("point is a list")
            else:
                rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",group)
                print "parameter is:",param
                return None

            # here: point should be list of floats/ints
            #print point
            if not len(point) == len(joint_names): # check dimension
                rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",group,len(joint_names),len(point))
                print "parameter is:",param
                return None

            for value in point:
                #print value,"type2 = ", type(value)
                if not ((type(value) is float) or (type(value) is int)): # check type
                    #print type(value)
                    rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",group)
                    print "parameter is:",param
                    return None
                rospy.logdebug("accepted value %f for %s",value,group)
            
            return point











