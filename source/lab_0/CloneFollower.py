#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import Utils # <----------- LOOK AT THESE FUNCTIONS ***************************

SUB_TOPIC = '/sim_car_pose/pose' # The topic that provides the simulated car pose
PUB_TOPIC = '/clone_follower_pose/pose' # The topic that you should publish to
MAP_TOPIC = 'static_map' # The service topic that will provide the map

# Follows the simulated robot around
class CloneFollower:

  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, follow_offset, force_in_bounds):
    # YOUR CODE HERE 
    self.follow_offset = follow_offset # Store the input params in self
    self.force_in_bounds = force_in_bounds # Store the input params in self
    self.map_img, self.map_info = Utils.get_map(MAP_TOPIC) # Get and store the map
                                  # for bounds checking
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher(PUB_TOPIC, queue_size=1)
    
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.sub = rospy.Subscriber(SUB_TOPIC, PoseStamped, self.update_pose, queue_size=1)
    
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  (This function is optional)
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):
    # YOUR CODE HERE
    x = rot[0,0] + rot[0,1] + trans[0]
    y = rot[1,0] + rot[1,1] + trans[1]
    return x,y
    
  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''  
  def update_pose(self, msg):
    # YOUR CODE HERE
      
    # Compute the pose of the clone
    # Note: To convert from a message quaternion to corresponding rotation matrix,
    #       look at the functions in Utils.py
    x = msg.Point.x
    y = msg.Point.y
    quant = msg.Quaternion

    theta = Utils.quaternion_to_angle(quant)
    rot = Utils.rotation_matrix(theta)

    x_, y_ = self.compute_follow_pose(trans=[self.follow_offset, 0], rot=rot)


    # Check bounds if required
    if self.force_in_bounds:
      # Functions in Utils.py will again be useful here
      if ((x >= self.map_img[1]) | (x <= 0) | (y >= self.map_img[0]) | (y <= 0)):
        x_, y_ = self.compute_follow_pose(trans=[-1*self.follow_offset, 0], rot=rot)
      
    # Setup the out going PoseStamped message
    ###### FIX THIS #######
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/map'

    world_pose = [x_,y_,rot]
    map_pose = Utils.world_to_map(world_pose, self.map_info)
    pose.Point.x = map_pose[0]
    pose.Point.y = map_pose[1]
    pose.Quaternion = map_pose[2]
    
    # Publish the clone's pose
    pub.publish(pose)

      
    
if __name__ == '__main__':
  follow_offset = 1.0 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced
  
  rospy.init_node('clone_follower', anonymous=True) # Initialize the node
  
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param("follow_offset")
  force_in_bounds = rospy.get_param("force_in_bounds")
  
  cf = CloneFollower(follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin() # Spin
  
