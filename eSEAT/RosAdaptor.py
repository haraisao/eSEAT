#
#
#
from __future__ import print_function
import sys
import os
import traceback

import yaml

try:
  import utils
except:
  from . import utils
#
#
try:
  import rospy
  import roslib
  import roslib.message
  import std_msgs.msg as std_msgs
  import geometry_msgs.msg as geometry_msgs
  import sensor_msgs.msg as sensor_msgs
  __ros_version__=1
  sys.path.append(os.path.realpath('ros/lib/site-packages'))
  ros_node_name=None
  os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = '' 
  from core import getGlobals, setGlobals
except:
  try:
    import rclpy
    import std_msgs.msg as std_msgs
    import geometry_msgs.msg as geometry_msgs
    import sensor_msgs.msg as sensor_msgs
    __ros_version__=2
    sys.path.append(os.path.realpath('ros2/lib/site-packages'))

    from .core import getGlobals, setGlobals
  except:
    __ros_version__=0
    raise(ImportError)

#
#  Functions
#
def setRosMaster(hostname=None):
  if not hostname : hostname = os.uname()[1]
  os.environ['ROS_MASTER_URI'] = 'http://%s:11311' % hostname

#
#
def initRosNode(name, anonymous=False):
  global ros_node_name
  if not ros_node_name:
    try:
      if __ros_version__ == 1:
        setRosMaster()
        rospy.init_node(name, anonymous=anonymous)
        ros_node_name=rospy.get_name()
        if ros_node_name == '/unnamed': ros_node_name=None
      elif __ros_version__ == 2:
        print("Sorry, not implemented")
        ros_node_name=None
      else:
        print("=== Unexpected error ===")
        ros_node_name=None

    except:
      print("=== Fail to ros_init ===")
      ros_node_name=None
  return ros_node_name

#
#
def ros_name():
  global ros_node_name
  return ros_node_name
  
#
#
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
def getRosVersion():
  global __ros_version__
  return __ros_version__

#
#
def createRate(hz):
  if __ros_version__ == 1:
    return rospy.Rate(hz)
  elif __ros_version__ == 2:
    print("Sorry, not implemented....")
  else:
    print("Unexpected error")

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
    elif self.type == 'Subscriber':
      self.createSubscriber(name, datatype, arg)

  #
  # 
  def createPublisher(self, name, datatype, size):
    if self.type == 'Publisher':
      if __ros_version__ == 1:
        dtype=getMsgClass(datatype)
        self._port=rospy.Publisher(name, dtype, queue_size=size)
      elif __ros_version__ == 2:
        print("Sorry, notimplement..")
      else:
        print("Unexpected error")
      
    return self._port

  #
  #
  def createSubscriber(self, name, datatype, callback):
    if self.type == 'Subscriber':
      if __ros_version__ == 1:
        dtype=getMsgClass(datatype)
        self._port=rospy.Subscriber(name, dtype, callback)
      elif __ros_version__ == 2:
        print("Sorry, notimplement..")
      else:
        print("Unexpected error")
      
    return self._port

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
      
        if __ros_version__ == 1:
          roslib.message.fill_message_args(msg, args)
          self._port.publish(msg)
        elif __ros_version__ == 2:
          print("Sorry, notimplement..")
        else:
          print("Unexpected error")

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
    if __ros_version__ == 1:
      return roslib.message.get_printable_message_args(self._port.data_class())
    elif __ros_version__ == 2:
      print("Sorry, notimplement..")
    else:
      print("Unexpected error")
    return None

  #
  #
  def createServer(self, srv_name, srv_type, srv_impl, fname):
    if srv_type.find('.') > 0:
      pkgname,srv = srv_type.split('.',1)
      exec_str="import %s.srv as %s" % (pkgname, pkgname)
      try:
        exec(exec_str, globals())
      except:
        pass

    if fname:
        utils.exec_script_file(fname, globals())

    resfunc=eval(srv_type+"Response")
    srv_func=lambda x :  resfunc(eval(srv_impl)(x))

    if __ros_version__ == 1:
      self._port=rospy.Service(srv_name, eval(srv_type), eval(srv_impl)) 
    elif __ros_version__ == 2:
      print("Sorry, notimplement..")
    else:
      print("Unexpected error")

    return self._port 
        
  #
  #
  def createClient(self, srv_name, srv_type):
    if srv_type.find('.') > 0:
      pkgname,srv = srv_type.split('.',1)
      exec_str="import %s.srv as %s" % (pkgname, pkgname)
      try:
        exec(exec_str, globals())
      except:
        pass

    if __ros_version__ == 1:
      self._port=rospy.ServiceProxy(srv_name, eval(srv_type)) 
    elif __ros_version__ == 2:
      print("Sorry, notimplement..")
    else:
      print("Unexpected error")

    return self._port 

  def callRosService(self, name, *args):
    try:
      if __ros_version__ == 1:
        return self._port(*args)
      elif __ros_version__ == 2:
        print("Sorry, notimplement..")
      else:
        print("Unexpected error")
      return None
    except:
      print("Error in callRosService " % name)
      return None
