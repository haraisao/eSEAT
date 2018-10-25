#!/usr/bin/python
#
#
import os
import sys


ROS_DIR = os.path.join(os.getcwd(), 'ros')
LIB_DIR = os.path.join(ROS_DIR, 'lib', 'site-packages')
PKG_DIR = os.path.join(ROS_DIR, 'packages')

def gen_package_dir(package):
  print("Generating {0} ...".format(package))
  package_path = os.path.join(PKG_DIR, package)
  output_path = os.path.join(LIB_DIR, package)
  try:
    os.makedirs(LIB_DIR)
  except:
    pass

  for gentype in ('msg', 'srv'):
    dirname=os.path.join(package_path, gentype,'*.{0}'.format(gentype))
    try:
      os.makedirs(dirname)
    except:
      pass
   
#
if __name__=='__main__':
  if len(sys.argv) < 2:
    print("Usage: gen_ros_dir.py <package name>")
  else:
    gen_package_dir(sys.argv[1])
