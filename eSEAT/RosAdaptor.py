#
#
#
from __future__ import print_function
import sys
import os
import traceback


import yaml

import rospy
import roslib
import roslib.message
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs

sys.path.append(os.path.realpath('ros/lib/site-packages'))

ros_node_name=None
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = '' 

from core import getGlobals, setGlobals

def setRosMaster(hostname=None):
  if not hostname : hostname = os.uname()[1]
  os.environ['ROS_MASTER_URI'] = 'http://%s:11311' % hostname


def initRosNode(name, anonymous=False):
  global ros_node_name
  if not ros_node_name:
    try:
      setRosMaster()
      rospy.init_node(name, anonymous=anonymous)
      ros_node_name=rospy.get_name()
      if ros_node_name == '/unnamed': ros_node_name=None

    except:
      print("=== Fail to ros_init ===")
      ros_node_name=None
  return ros_node_name

def ros_name():
  global ros_node_name
  return ros_node_name
  
def getMsgClass(datatype):
  try:
    dtype=eval(datatype.replace('/','.'),getGlobals())
    return dtype
  except:
    try:
      pkg, klass=datatype.split('/')
      exec("import "+pkg+".msg as "+pkg , getGlobals())
      dtype=eval(datatype.replace('/','.'),getGlobals())
      return dtype
    except:
      traceback.print_exc()
      return None
   
        

#
#
class RosAdaptor(object):
  def __init__(self,name,typ):
    self.name=name
    self.type=typ
    self._port=None

  #
  #
  def send(self, name, val):
    if self.name == name:
      if self.type == 'Publisher':
        self.publish(val)
    else:
      print("Error: mismatch topic name")

  def create(self, name, datatype, arg):
    if self.type == 'Publisher':
      self.createPublisher(name, datatype, arg)
    if self.type == 'Subscriber':
      self.createSubscriber(name, datatype, arg)

  #
  # 
  def createPublisher(self, name, datatype, size):
    if self.type == 'Publisher':
      dtype=getMsgClass(datatype)
      self._port=rospy.Publisher(name, dtype, queue_size=size)

  #
  #
  def createSubscriber(self, name, datatype, callback):
    if self.type == 'Subscriber':
      dtype=getMsgClass(datatype)
      self._port=rospy.Subscriber(name, dtype, callback)

  #
  #
  def publish(self, val):
    try:
      if isinstance(val, self._port.data_class) :
        self._port.publish(val)
      elif  self._port.data_class == std_msgs.String :
        self._port.publish(val)
      else:
        msg=self._port.data_class()
        if type(val) == str:
          val = yaml.load(val)
        args=[]
        for x in val:
          if type(x) == str:
            args.append(yaml.load(x)) 
          else:
            args.append(x) 
      
        roslib.message.fill_message_args(msg, args)
        self._port.publish(msg)

    except:
      traceback.print_exc()
      pass

  #
  #
  def newMessage(self):
    return self._port.data_class()

  #
  #
  def getMessageSlots(self):
    return roslib.message.get_printable_message_args(self._port.data_class())
