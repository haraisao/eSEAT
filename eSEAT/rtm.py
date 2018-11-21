#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''eSEAT (Extened Simple Event Action Transfer)

Copyright (C) 2009-2014
    Yosuke Matsusaka and Isao Hara
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST), Japan
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
import utils

from viewer import OutViewer

if os.getenv('SEAT_ROOT') :
  rootdir=os.getenv('SEAT_ROOT')
else:
  rootdir='/usr/local/eSEAT'

sys.path.append(rootdir)
sys.path.append(os.path.join(rootdir,'/libs'))
sys.path.append(os.path.join(rootdir,'/3rd_party'))

opts = None

#
#
########
#  for OpenRTM-aist
import OpenRTM_aist
import omniORB
from RTC  import *

########################################
#  eSEAT_Core
#
from core import eSEAT_Core,eSEAT_Gui,getGlobals,setGlobals
import SeatmlParser

__version__="2.5"

#########################################################################
#
#  Sprcification of eSEAT
#
eseat_spec = ["implementation_id", "eSEAT",
             "type_name",         "eSEAT",
             "description",       __doc__.encode('UTF-8'),
             "version",           __version__,
             "vendor",            "AIST",
             "category",          "OpenHRI",
             "activity_type",     "DataFlowComponent",
             "max_instance",      "1",
             "language",          "Python",
             "lang_type",         "script",
             "conf.default.scriptfile", "None",
             "conf.default.scorelimit", "0.0",
#             "conf.__widget__.scorelimit", "slider",
             "exec_cxt.periodic.rate", "1",
             ""]

#########################################################################
#
# DataListener 
#   This class connected with DataInPort
#
class eSEATDataListener(OpenRTM_aist.ConnectorDataListenerT):
    def __init__(self, name, type, obj):
        self._name = name
        self._type = type
        self._obj = obj
    
    def __call__(self, info, cdrdata):
        data = OpenRTM_aist.ConnectorDataListenerT.__call__(self,
                        info, cdrdata, instantiateDataType(self._type))
        self._obj.onData(self._name, data)

#########################################################################
#
#
class RtcLogger:
    def __init__(self, name):
        self._logger = OpenRTM_aist.Manager.instance().getLogbuf(name)

    def setFlag(self, flag):
        if flag:
            self._logger.setLogLevel('INFO')
        else:
            self._logger.setLogLevel('ERROR')

    def info(self, msg):
        self._logger.RTC_INFO(msg)

    def error(self, msg):
        self._logger.RTC_ERROR(msg)

    def warn(self, msg):
        self._logger.RTC_WARN(msg)


#########################################################################
#
#  Class eSEAT (RTC)
#
class eSEAT(OpenRTM_aist.DataFlowComponentBase, eSEAT_Gui, eSEAT_Core):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        eSEAT_Core.__init__(self)
        eSEAT_Gui.__init__(self)

        self.manager = None
        self.activated=False
        self._consumer = {}
        self._ConsumerPort = {}
        self._ProviderPort = {}
        self._datatype = {}
        self._on_timeout = -1
        self.rate_hz=0 
        self.rtsh=None

    def setRtsh(self, ns):
        self.rtsh=ns

    def getRtsh(self):
        return self.rtsh

    def exit(self):
        try:
            eSEAT_Core.exit_comp(self)
            return OpenRTM_aist.DataFlowComponentBase.exit(self)
        except:
            return RTC_OK

    def terminate(self):
        self.exit() 

    ##########################################################
    #  E v e n t   H a n d l e r 
    #  onInitialize
    #
    def onInitialize(self):
        OpenRTM_aist.DataFlowComponentBase.onInitialize(self)
        self._logger = RtcLogger(self._properties.getProperty("instance_name"))
        self._logger.info("eSEAT (Extended Simple Event Action Transfer) version " + __version__)
        self._logger.info("Copyright (C) 2009-2014 Yosuke Matsusaka and Isao Hara")
        self.bindParameter("scriptfile", self._scriptfile, "None")
        self.bindParameter("scorelimit", self._scorelimit, "0.0")

        return RTC_OK

    #
    # for RTC
    #  onActivated
    #
    def onActivated(self, ec_id):
        self.activated = True
        self.processActivated()
        self.resetTimer()
        #print (self.currentstate)
        return RTC_OK

    #
    # for RTC
    #  onDeactivated
    #
    def onDeactivated(self, ec_id):
        self.processDeactivated()
        self.processDeactivated('all')
        self.activated = False
        return RTC_OK

    #
    # for RTC
    #  onFinalize
    #
    def onFinalize(self):
        try:
            OpenRTM_aist.DataFlowComponentBase.onFinalize(self)
            self.finalizeSEAT()
        except:
            pass

        return RTC_OK

    #
    # for RTC
    #  onShutdown
    #
    def onShutdown(self, ec_id):
        return RTC_OK
    #
    # for RTC
    #  onExecute
    #
    def onExecute(self, ec_id):
        OpenRTM_aist.DataFlowComponentBase.onExecute(self, ec_id)
        if self.isTimeout() :
            res=self.processTimeout()
            if res :
                return RTC_OK
            else:
                self._on_timeout = -1
        self.processExec()
        self.processExec('all')
        return RTC_OK

    #
    # for RTC
    #  onData: this method called in comming data
    #
    def onData(self, name, data):
        if self.activated :
            self.resetTimer()
            try:
                if isinstance(data, TimedString):
                    data.data = data.data.decode('utf-8')
                    self.processResult(name, data.data)
                    self.processOnDataIn(name, data)
                elif isinstance(data, TimedWString):
                    data.data = data.data
                    self.processResult(name, data.data)
                    self.processOnDataIn(name, data)
                else:
                    eSEAT_Core.onData(self, name, data)
            except:
                self._logger.error(traceback.format_exc())
        else:
            pass



    ##############################################
    #
    #  RTC state control funtion
    #
    def activate(self):
      execContexts = self.get_owned_contexts()
      execContexts[0].activate_component(self.getObjRef())

    def deactivate(self):
      execContexts = self.get_owned_contexts()
      execContexts[0].deactivate_component(self.getObjRef())

    ##########################################################
    # Create PORT
    #
    # Create the InPort of RTC
    #
    def createInPort(self, name, type=TimedString):
        self._logger.info("create inport: " + name)
        self._datatype[name]=type
        self._data[name] = instantiateDataType(type)
        self._port[name] = OpenRTM_aist.InPort(name, self._data[name])
        self._port[name].addConnectorDataListener(
                            OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                            eSEATDataListener(name, type, self))
        self.registerInPort(name, self._port[name])

    #
    # Create the OutPort of RTC
    #
    def createOutPort(self, name, type=TimedString):
        self._logger.info("create outport: " + name)
        self._datatype[name]=type
        self._data[name] = instantiateDataType(type)
        self._port[name] = OpenRTM_aist.OutPort(name, self._data[name],
                                   OpenRTM_aist.RingBuffer(8))
        self.registerOutPort(name, self._port[name])

    #
    # Create and Register DataPort of RTC
    #
    def createDataPort(self, name, dtype, inout):
        if inout == 'rtcout':
            self.adaptortype[name] = self.getDataType(dtype)
            self.createOutPort(name, self.adaptortype[name][0])
            self.adaptors[name] = self

        elif inout == 'rtcin':
            self.adaptortype[name] = self.getDataType(dtype)
            self.createInPort(name, self.adaptortype[name][0])
            self.adaptors[name] = self
        else:
            return False

        return True

    #
    # Service Port
    #
    def createServicePort(self, name, if_name, type_name, klass, srv_type):
        if srv_type == 'provider':
            self.createProviderPort(name, if_name, type_name, klass)
        elif srv_type == 'consumer':
            self.createConsumerPort(name, if_name, type_name, klass)
        else:
            return False
        return True
    #
    # Create Provider
    def createProviderPort(self, name, if_name, type_name, impl_class):
        self._ProviderPort[name] = OpenRTM_aist.CorbaPort(name)
        service = impl_class()
        service._seat = self
        self._ProviderPort[name].registerProvider(if_name, type_name, service )
        self.addPort(self._ProviderPort[name])
        return RTC_OK
    #
    # Create Consumer
    def createConsumerPort(self, name, if_name, type_name, if_type):
        self._ConsumerPort[name] = OpenRTM_aist.CorbaPort(name)
        self._consumer[name] = OpenRTM_aist.CorbaConsumer(interfaceType=if_type)
        self._ConsumerPort[name].registerConsumer(if_name, type_name, self._consumer[name])
        self.addPort(self._ConsumerPort[name])
        return RTC_OK
    #
    #
    def getServicePtr(self, name):
        if name in self._consumer:
            return self._consumer[name]._ptr()
        else:
            return None
    #
    #
    def callServiceAsync(self, name, func):
        async_call = OpenRTM_aist.Async_tInvoker(self.getServicePtr(name), func)
        async_call.invoke()
        return async_call
    
    #
    #
    def callService(self, name, m_name, *val):
        try:
            objref=self._consumer[name]._ptr()
            return eval('objref.'+m_name)(*val)
            #return self._consumer[name]._ptr().__getattribute__(m_name)(*val)
        except:
            traceback.print_exc()
            return None
        
    #
    #    Create communication adaptor
    #
    def createAdaptor(self, compname, tag, env=getGlobals()):
        try:
            name = str(tag.get('name'))
            type = tag.get('type')

            if type == 'rtcin' or type == 'rtcout' :
                module=tag.get('datatype').split('.')
                if len(module) > 1 and not module[0] in env:
                    exec("import "+module[0], env)              
                return self.createDataPort(name, tag.get('datatype') ,type)

            elif type == 'provider' or type == 'consumer' :
                if_type, if_name=tag.get('interface').split('|')
                if_class=tag.get('if_class') 
                if not if_class: if_class=if_name

                module=if_class.split('.')
                if len(module) > 1 and not module[0] in env:
                    exec("import "+module[0], env)

                impl_file=tag.get('impl_file')
                if impl_file:
                    utils.exec_script_file(impl_file, env)

                return self.createServicePort(name, if_name, if_type, eval(if_class, env), type)
            else:
                 return eSEAT_Core.createAdaptor(self, compname, tag)
        except:
            self._logger.error(u"invalid parameters: " + type + ": " + name)
            traceback.print_exc()
            #print( traceback.format_exc() )
            return -1

        return 1
    #
    # Disconnect all connections
    #
    def disconnectAll(self):
      for p in self._port.keys():
          if isinstance(self._port[p], OpenRTM_aist.PortBase) :
              self._port[p].disconnect_all()

    #######################
    #  Get DataType
    #
    def getDataType(self, s):
        if len(s) == 0         : return (TimedString, 0)
        seq = False

        if s[-3:] == "Seq"     : seq = True

        dtype = str
        if s.count("WString")  : dtype = unicode
        elif s.count("String") : dtype = str
        elif s.count("Float")  : dtype = float
        elif s.count("Double") : dtype = float
        elif s.count("Short")  : dtype = int
        elif s.count("Long")   : dtype = int
        elif s.count("Octet")  : dtype = int
        elif s.count("Char")   : dtype = str
        elif s.count("Boolean"): dtype = int
        else                   : dtype = eval("%s" % s)

        return (eval("%s" % s), dtype, seq)

    #
    #
    def newData(self,name):
        if self.adaptors[name] == self:
          return instantiateDataType(self._datatype[name])
        else:
          return eSEAT_Core.newData(self, name)

    #
    #
    def isNew(self,name):
        try:
            return self._port[name].isNew()
        except:
            return False

    #
    #
    def writeData(self,name):
        try:
            self._port[name].write()
        except:
            pass

    #
    #
    def getData(self,name):
        try:
            return self._data[name]
        except:
            return None

    #
    #
    def readData(self,name):
        try:
            return self._port[name].read()
        except:
            return None

    def readAllData(self,name):
        try:
            res=[]
            while self._port[name].isNew():
                res.append(self._port[name].read())
            return res
        except:
            return []

    def get_time(self):
        return time.time()

    ############ End of RTC functions


#########################################################################
#
# eSEAT Manager ( RTC manager )
#
class eSEATManager:
    def __init__(self, mlfile=None):
        self.comp = None
        self.run_as_daemon = False
        self.naming_format = ""
        self.viewer = None

        if mlfile is None:
            opts, argv = self.parseArgs()
            if argv == -1:
                raise Exception("Error in __init__")
        else:
            #argv = []
            #opts = None
            opts, argv = self.parseArgs(False)
            self._scriptfile = mlfile

        #
        #  check configuration file...
        mgrconf = OpenRTM_aist.ManagerConfig(argv)
        if not mgrconf.findConfigFile():
           argv.insert(1, "-f") 
           argv.insert(2, rootdir + "/rtc.conf") 


        #
        #
        if opts:
            self.run_as_daemon = opts.run_as_daemon
            self.naming_format = opts.naming_format

        if self.run_as_daemon :
            daemonize()

        self.manager = OpenRTM_aist.Manager.init(argv)

        if self.naming_format:
            self.manager._config.setProperty("naming.formats", self.naming_format)

        #
        # create Component
        self.manager.setModuleInitProc(self.moduleInit)
        self.manager.activateManager()

        #
        #  set Instance Name
        instance_name = self.comp.getProperties().getProperty("naming.names")
        instance_name = SeatmlParser.formatInstanceName(instance_name)
        self.comp.setInstanceName(instance_name)

        self.comp.setRtsh(Rtc_Sh(self.manager.getORB()))
        self.comp.seat_mgr=self

        #
        # Gui stdout
        if opts and opts.run_out_viewer:
            #print ("CreateViewer")
            self.viewer = OutViewer()

    #
    #  Parse command line option...
    def parseArgs(self, flag=True):
        encoding = locale.getpreferredencoding()
        sys.stdout = codecs.getwriter(encoding)(sys.stdout, errors = "replace")
        sys.stderr = codecs.getwriter(encoding)(sys.stderr, errors = "replace")

        parser = utils.MyParser(version=__version__, usage="%prog [seatmlfile]",
                                description=__doc__)

        parser.add_option('-f', '--config-file', dest='config_file', type="string",
                            help='apply configuration file')

        parser.add_option('-n', '--name', dest='naming_format', type="string",
                            help='set naming format' )

        parser.add_option('-d', '--daemon', dest='run_as_daemon', action="store_true",
                            help='run as daemon' )

        parser.add_option('-v', '--viewer', dest='run_out_viewer', action="store_true",
                            help='create output window' )

        parser.add_option('-o', '--option', dest='option', action='append',
                      default=None,
                      help='specify custom configuration parameter')

        try:
            opts, args = parser.parse_args()
        except (optparse.OptionError, e):
            print('OptionError:', e, file=sys.stderr)
            #sys.exit(1)
            return [None, -1]

        self._scriptfile = None
        if(len(args) > 0):
            self._scriptfile = args[0]

        if opts.naming_format:
           sys.argv.remove('-n')
           sys.argv.remove(opts.naming_format)

        if opts.run_as_daemon:
           sys.argv.remove('-d')

        if opts.run_out_viewer:
           sys.argv.remove('-v')

        if len(args) == 0 and flag:
            parser.error("wrong number of arguments")
            #sys.exit(1)
            return [None, -1]
        
        return [opts, sys.argv]

    #
    #  Initialize the eSEAT-RTC
    #
    def moduleInit(self, manager):
        profile = OpenRTM_aist.Properties(defaults_str=eseat_spec)
        manager.registerFactory(profile, eSEAT, OpenRTM_aist.Delete)
        self.comp = manager.createComponent("eSEAT")
        self.comp.manager = manager
        manager.unregisterComponent(self.comp)

        if self._scriptfile :
            ret = self.comp.loadSEATML(self._scriptfile)
            if ret : raise Exception("Error in moduleInit")

        naming_formats = self.comp.getProperties().getProperty("naming.formats")
        naming_names = manager.formatString(naming_formats, self.comp.getProperties())
        self.comp.getProperties().setProperty("naming.names",naming_names)
        manager.registerComponent(self.comp)


    #
    #  Start Manager
    #
    def start(self):
        self.comp.startRosService()

        if (isinstance(self.comp, eSEAT_Gui) and self.comp.hasGUI() ) or self.viewer :
            self.manager.runManager(True)

            # GUI part
            res = self.comp.startGuiLoop(self.viewer)
            if res == 0:
              self.comp.disconnectAll()
              # Shutdown Component
              try:
                 self.manager.shutdown()
              except:
                 sys.exit(1)
              return
            else:
              self.manager.runManager()
        else:
            self.manager.runManager()

    #
    #
    def exit(self):
        try:
            self.comp.exit_comp()
            self.manager.shutdown()
        except:
            pass
        print( "....eSEAT Manager shutdown" )
        if self.run_as_daemon:
          os._exit(1)
        else:
          os._exit(1)


#########################################################################
#   F U N C T I O N S
#
#  DataType for CORBA
#
def instantiateDataType(dtype):
    if isinstance(dtype, int) : desc = [dtype]
    elif isinstance(dtype, tuple) : desc = dtype
    else : 
        desc=omniORB.findType(dtype._NP_RepositoryId) 

    if desc[0] in [omniORB.tcInternal.tv_alias ]: return instantiateDataType(desc[2])

    if desc[0] in [omniORB.tcInternal.tv_short, 
                   omniORB.tcInternal.tv_long, 
                   omniORB.tcInternal.tv_ushort, 
                   omniORB.tcInternal.tv_ulong,
                   omniORB.tcInternal.tv_boolean,
                   omniORB.tcInternal.tv_char,
                   omniORB.tcInternal.tv_octet,
                   omniORB.tcInternal.tv_longlong,
                   omniORB.tcInternal.tv_enum
                  ]: return 0

    if desc[0] in [omniORB.tcInternal.tv_float, 
                   omniORB.tcInternal.tv_double,
                   omniORB.tcInternal.tv_longdouble
                  ]: return 0.0

    if desc[0] in [omniORB.tcInternal.tv_sequence, 
                   omniORB.tcInternal.tv_array,
                  ]: return []


    if desc[0] in [omniORB.tcInternal.tv_string ]: return ""
    if desc[0] in [omniORB.tcInternal.tv_wstring,
                   omniORB.tcInternal.tv_wchar
                  ]: return u""

    if desc[0] == omniORB.tcInternal.tv_struct:
        arg = []
        for i in  range(4, len(desc), 2):
            attr = desc[i]
            attr_type = desc[i+1]
            arg.append(instantiateDataType(attr_type))
        return desc[1](*arg)
    return None

#
#
def main_rtm(mlfile=None, daemon=False):
    try:
        import signal

        def sig_handler(signum, frame):
          seatmgr.exit() 

        signal.signal(signal.SIGINT, sig_handler)        

        if daemon : daemonize()
        seatmgr = eSEATManager(mlfile)
        seatmgr.start()
    except:
        traceback.print_exc()
        pass

    print ( "...Terminate." )
    try:
      if seatmgr.run_as_daemon:
        os._exit(1)
      else:
        os._exit(1)
        #sys.exit(1)
    except:
      os._exit(1)
      #sys.exit(1)

#########################
import CosNaming 
from CorbaNaming import *
import SDOPackage
import omniORB.URI
import string

class Rtc_Sh:
  def __init__(self, orb, server_name='localhost'):
    self.orb=orb
    self.name=server_name
    self.naming=CorbaNaming(self.orb, self.name)
    self.maxlen=20
    self.object_list={}
    self.getRTObjectList()

  def resolveRTObject(self, name):
    if name.count(".rtc") == 0 : name = name+".rtc"
    ref=self.naming.resolveStr(name)
    return ref._narrow(RTObject)

  def unbind(self, name):
    self.naming.unbind(name)
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

    name = prefix + omniORB.URI.nameToString(bind.binding_name)
    if bind.binding_type == CosNaming.nobject:
      if bind.binding_name[0].kind == "rtc":
        obj = name_context.resolve(bind.binding_name)
        self.object_list[name] = obj
        try:
          obj = obj._narrow(RTObject)
          res = [[name, obj]]
        except:
          obj = None
      else:
        self.object_list[name] = None
    else:
      ctx = name_context.resolve(bind.binding_name)
      ctx = ctx._narrow(CosNaming.NamingContext)
      res = self.getRTObjectList( name_context, parent)
    return res

  def refreshObjectList(self):
    self.object_list = {}
    self.getRTObjectList()

  def getPorts(self, name):
    res=[]
    if name in self.object_list:
      self.refreshObjectList()

    if name.count(".rtc") == 0 : name = name+".rtc"
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

  def activate(self, name):
    obj=self.resolveRTObject(name)
    ec=obj.get_owned_contexts()[0]
    ec.activate_component(obj)
    return None
      
  def deactivate(self, name):
    obj=self.resolveRTObject(name)
    ec=obj.get_owned_contexts()[0]
    ec.deactivate_component(obj)
    return None

  def terminate(self, name):
    obj=self.resolveRTObject(name)
    obj.exit()
    return None
      
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
#     

#########################################################################
#
#  M A I N 
#
if __name__=='__main__':
  main_rtm()
