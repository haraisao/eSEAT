#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''

RtcSh
Copyright (C) 2018
    Isao Hara,AIST,Japan
    All rights reserved.
  Licensed under the MIT License (MIT)
  http://www.opensource.org/licenses/MIT
'''

############### import libraries
from __future__ import print_function
import sys
import os
import getopt
import codecs
import locale
import time
import signal
import re
import traceback
import optparse
import threading
import subprocess
#import utils

import cmd

#########################
from RTC import *
import CosNaming 
from CorbaNaming import *
import SDOPackage
from omniORB import CORBA,URI,any
import string

#
#
class Rtc_Sh:
  def __init__(self, orb=None, server_name='localhost'):
    if orb is None:
      self.orb = CORBA.ORB_init(sys.argv)
    else:
      self.orb=orb
    self.name=server_name
    self.naming=CorbaNaming(self.orb, self.name)
    self.maxlen=20
    self.object_list={}
    self.current_ctx=""
    #self.getRTObjectList()

  def resolveRTObject(self, name):
    try:
      if name.count(".rtc") == 0 : name = name+".rtc"
      ref=self.naming.resolveStr(name)
      ref._non_existent()
      return ref._narrow(RTObject)
    except:
      #traceback.print_exc()
      return None

  def unbind(self, name):
    self.naming.unbind(name)
    print("Unbind :", name)
    return

  def clearObjectList(self):
    self.object_list={}

  def getRTObjectList(self, name_context=None, parent=""):
    res=[]
    if name_context is None:
      name_context = self.naming._rootContext
    binds, bind_i = name_context.list(self.maxlen)
    for bind in binds:
      res = res + self.resolveBindings(bind, name_context, parent)

    if bind_i :
      tl = bind_i.next_n(self.maxlen)
      while tl[0]:
        for bind in tl[1] :
           res = res + self.resolveBindings(bind, name_conext, parent)
        tl = bind_i.next_n(self.maxlen)
    return res

  def resolveBindings(self, bind, name_context, parent):
    res = []
    prefix=parent

    if parent :
      prefix += "/"

    name = prefix + URI.nameToString(bind.binding_name)
    if bind.binding_type == CosNaming.nobject:
      if bind.binding_name[0].kind == "rtc":
        obj = name_context.resolve(bind.binding_name)
        try:
          obj._non_existent()
          obj = obj._narrow(RTObject)
          res = [[name, obj]]
          self.object_list[name] = obj
        except:
          obj = None
          res = [[name, obj]]
      else:
        pass
        #self.object_list[name] = None
    else:
      ctx = name_context.resolve(bind.binding_name)
      ctx = ctx._narrow(CosNaming.NamingContext)
      parent = name
      res = self.getRTObjectList( ctx, parent)
    return res

  def refreshObjectList(self):
    self.object_list = {}
    return self.getRTObjectList()

  def getPorts(self, name):
    res=[]
    if name.count(".rtc") == 0 : name = name+".rtc"
    if not (name in self.object_list):
      self.refreshObjectList()

    if name in self.object_list:
      port_ref = self.object_list[name].get_ports()
      for p in port_ref:
        pp = p.get_port_profile()
        pprof =  nvlist2dict(pp.properties)
        res.append( (pp.name, pprof))
    else:
      print("No such RTC:", name)
    return res

  def getPortRef(self, name, port):
    res=[]
    if name in self.object_list:
      self.refreshObjectList()

    if name.count(".rtc") == 0 : name = name+".rtc"

    if name in self.object_list:
      port_ref = self.object_list[name].get_ports()
      for p in port_ref:
        pp = p.get_port_profile()
        if port == pp.name.split('.')[-1]:
          return p
    else:
      print("No such port:", name, ":", port)
    return None

  def getConnectors(self, name, port):
    port_ref=self.getPortRef(name, port)
    if port_ref:
      cons = port_ref.get_connector_profiles()
      return cons
    return None
 
  def getConnectionInfo(self, con):
    ports = [(con.ports[0].get_port_profile()).name, (con.ports[1].get_port_profile()).name]
    res={'name': con.name, 'ports': ports, 'id': con.connector_id }
    return res

  def getConnections(self, name, port):
    res = []
    cons = self.getConnectors(name, port)
    if cons:
      for c in cons:
        res.append(self.getConnectionInfo(c))
    return res

  def find_connection(self, portname1, portname2):
    try:
      name1, port1 = portname1.split(":")
      name2, port2 = portname2.split(":")

      p1=self.getPortRef(name1, port1)
      p2=self.getPortRef(name2, port2)

      cons  = self.getConnectors(name1, port1)
      cons2 = self.getConnectors(name2, port2)
      if cons and  cons2 :
        for c in cons:
          for c2 in cons2:
            if c.connector_id == c2.connector_id:
              return c
      return False
    except:
      traceback.print_exc()
      return None

  def connect(self, portname1, portname2, service=False):
    if service:
      con_prof = {'port.port_type':'CorbaPort' }
    else:
      con_prof={'dataport.dataflow_type':'push',
              'dataport.interface_type':'corba_cdr' ,
              'dataport.subscription_type':'flush'}

    chk = self.find_connection(portname1, portname2)
    if chk is None:
        return None
    if chk :
       print("Conntction exists:", chk.connector_id)
       return 
    try:
      name1, port1 = portname1.split(":")
      name2, port2 = portname2.split(":")
      p1=self.getPortRef(name1, port1)
      p2=self.getPortRef(name2, port2)
      if p1 and p2:
        name=string.join([name1, port1, name2, port2], '_')
        prof_req=ConnectorProfile(name, "", [p1, p2], dict2nvlist(con_prof)) 
        res, prof=p1.connect(prof_req)
      else:
        res="Error in connect"
    except:
      res="Error"
    print(res)
    return

  def disconnect(self, portname1, portname2):
    try:
      con=self.find_connection(portname1, portname2)
      if con is None or not con:
        print("No such connrction:", portname1, portname2)
       
      con.ports[0].disconnect(con.connector_id)
      print("Sucess to disconnect:", portname1, portname2)
    except:
      print("Fail to disconnect:", portname1, portname2)

  def getEC(self, name):
    obj=self.resolveRTObject(name)
    if obj :
      ec=obj.get_owned_contexts()[0]
      return ec
    else:
      return None
      
  def activate(self, name):
    obj=self.resolveRTObject(name)
    if obj :
      ec=obj.get_owned_contexts()[0]
      ec.activate_component(obj)
    return None
      
  def deactivate(self, name):
    obj=self.resolveRTObject(name)
    if obj :
      ec=obj.get_owned_contexts()[0]
      ec.deactivate_component(obj)
    return None

  def get_component_state(self, name):
    obj=self.resolveRTObject(name)
    ec=obj.get_owned_contexts()[0]
    return ec.get_component_state(obj)

  def terminate(self, name):
    obj=self.resolveRTObject(name)
    obj.exit()
    return None

#
#
class RtCmd(cmd.Cmd):
  #intro="Welcome to RtCmd"
  prompt="==> "
  file=None

  def __init__(self, rtsh=None, once=False):
    cmd.Cmd.__init__(self)
    if rtsh is None:
      self.rtsh=Rtc_Sh()
    else:
      self.rtsh=rtsh
    self.onecycle=once
    self.end=False

  def do_echo(self, arg, arg2=None):
    print(arg, arg2)
    return self.onecycle

  def do_list(self, arg):
    num=0
    argv=arg.split()
    if len(argv) > 1 and argv[1] == '-r':
      rtsh.refreshRTObjectList()
  
    print("===== RTCs =====")
    res = self.rtsh.getRTObjectList()
    for n in res:
      num += 1
      if n[1]:
        stat=self.rtsh.get_component_state(n[0])
        if stat == ACTIVE_STATE:
          print(num, ":", n[0], "*")
        else :
          print(num, ":", n[0])
      else:
        print(num, ":[", n[0], "]")
    print("")
    return self.onecycle

  def do_get_ports(self, arg):
    num=0
    ports = self.rtsh.getPorts(arg)
    print("====== Ports(%s) ======" % arg)
    for pp in ports:
      num += 1
      print(num, ":", pp[0].split('.')[1])
      for k in pp[1]:
         print("   ", k,":", pp[1][k])
            
    print("")
    return self.onecycle

  def do_get_connection(self, arg):
    argv=arg.split()
    cons = self.rtsh.getConnections(argv[0], argv[1])
    num=0
    if cons:
      for x in cons:
        print(num, ":", cons)
        num += 1
    else:
      print("No connection")
    return self.onecycle

  def do_disconnect(self, arg):
    argv=arg.split()
    self.rtsh.disconnect(argv[1], argv[2])
    return self.onecycle

  def do_connect(self, arg):
    argv=arg.split()
    self.rtsh.connect(argv[1], argv[2])
    return self.onecycle

  def do_activate(self, arg):
    self.rtsh.activate(arg)
    return self.onecycle

  def do_deactivate(self, arg):
    self.rtsh.deactivate(arg)
    return self.onecycle

  def do_get_state(self, arg):
    stat=self.rtsh.get_component_state(arg)
    print("State:", arg,":", stat)
    return self.onecycle

  def do_terminate(self, arg):
    self.rtsh.terminate(arg)
    return self.onecycle

  def do_unbind(self, arg):
    self.rtsh.unbind(arg)
    return self.onecycle

  def do_bye(self, arg):
    print('...BYE')
    self.close()
    self.end=True
    return True

  # ----- record and playback -----
  def do_record(self, arg):
    self.file = open(arg, 'w')

  def do_playback(self, arg):
    self.close()
    with open(arg) as f:
      self.cmdqueue.extend(f.read().splitlines())

  def precmd(self, line):
    #line = line.lower()
    if self.file and 'playback' not in line:
      print(line, file=self.file)
    return line

  def close(self):
    if self.file:
      self.file.close()
      self.file = None


def nvlist2dict(nvlist):
  res={}
  for v in nvlist:
    res[v.name] = v.value.value()
  return res

def dict2nvlist(dict) :
  import omniORB.any
  rslt = []
  for tmp in dict.keys() :
    rslt.append(SDOPackage.NameValue(tmp, omniORB.any.to_any(dict[tmp])))
  return rslt

#########################################################################
#
#  M A I N 
#
if __name__=='__main__':
  #while True:
    RtCmd().cmdloop()
