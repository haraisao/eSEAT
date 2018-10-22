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
import traceback
import subprocess

import time
import utils
import yaml

from collections import OrderedDict

########
#  for OpenRTM-aist
#import OpenRTM_aist
#import omniORB
#from RTC  import *

########
# XML Parser of Julius result
from bs4  import BeautifulSoup

#########
#  GUI etc.
try:
    from Tkinter import * 
    from ScrolledText import ScrolledText 
except:
    from tkinter import *
    from tkinter.scrolledtext import ScrolledText
 
####### for ROS
try:
  from RosAdaptor import *
except:
  rospy=None


###############################################################
#  Global Variables
#
def getGlobals():
    return globals()

def setGlobals(name, val):
    globals()[name] = val

#########
#  WebAdaptor
from SocketAdaptor import SocketAdaptor 

#########
#  WebAdaptor
from WebAdaptor import WebSocketServer,CometReader,parseQueryString

from Task import State, TaskGroup

###############################################################
#
#  execute seatml parser files
#
from SeatmlParser import SEATML_Parser,convertDataType

###############################################################
#
# Dummy logger
#
class SeatLogger:
    #
    #
    def __init__(self, name):
        self._name = name
        self._flag = True
    #
    #
    def setFlag(self, b):
        self._flag = b
    #
    #
    def info(self, msg):
        if self._flag :
            print ("INFO:"+self._name+" "+msg)
    #
    #
    def error(self, msg):
        print ("ERROR:"+self._name+" "+msg, file=sys.stderr)
    #
    #
    def warn(self, msg):
        print ("WARN:"+self._name+" "+msg, file=sys.stderr)


##################################################################
#
#  Class eSEAT_Core
#
class eSEAT_Core:
    #
    #
    def __init__(self):
        if hasattr(sys, "frozen"):
            self._basedir = os.path.dirname(unicode(sys.executable, sys.getfilesystemencoding()))
        else:
            self._basedir = os.path.dirname(__file__)
        self.parser = SEATML_Parser(self)
        self.states = OrderedDict()
        #self.keys = {}
        self.regkeys = {}
        self.statestack = []
        self.currentstate = "start"
        self.adaptors = {}
        self.adaptortype = {}

        self._data = {}
        self._port = {}
        self.popen = []

        self.init_state = None
        self._scriptfile = ["None"]
        self._scorelimit = [0.0]
        self.webServer = None
        self.root = None

        self.last_on_data = 0

        self._logger = SeatLogger("eSEAT")

        setGlobals('seat', self)

        self.name="eSEAT"
        self.ros_node=None
        self.ros_anonymous=False

    #
    #
    def exit_comp(self):
        print ("Call eSEAT_Core.exit")
        if self.webServer :
            self.webServer.terminate()
            print ("Call terminate()")
            time.sleep( 1 )
        print ("Call eSEAT_Core.exit.. done")
        return

    ##### Other Adaptors
    #
    # Create the Raw Socket
    #
    def createSocketPort(self, name, host, port):
        self.adaptors[name] = SocketAdaptor(self, name, host, port)

    #
    # Create the Web Server Port
    #
    def createWebAdaptor(self, name, port, index, whost="", dirname="html"):
        if self.webServer  is None:
            self.adaptors[name] = WebSocketServer(CometReader(self,dirname), name, "", port, index)
            if self.adaptors[name].bind_res != 1 :
                print ("=== Bind ERROR ===")
                os._exit(1)

            self.adaptors[name].start()
            self.webServer = self.adaptors[name]
            if whost :
                whosts = whost.split(',')
                for x in whosts:
                    if x :
                        self.webServer.appendWhiteList(x.strip())
        else:
            self._logger.info(u"Failed to create Webadaptor:" + name + " already exists")

    #
    #
    #   for ROS
    def initRosNode(self):
      if rospy:
        self.ros_node=initRosNode(self.name, self.ros_anonymous)
      else:
        print("Unsupport rospy")

    #
    #
    def createRosPublisher(self, name, datatype, size):
      if rospy:
        self.initRosNode()
        if self.ros_node:
          self.adaptors[name]=RosAdaptor(name, 'Publisher')
          self.adaptors[name].create(name, datatype, size)

    #
    #
    def ros_publish(self, name, val):
      try:
        self.adaptors[name].publish(val)
      except:
        print("Fail to ros_publish %s" % name)

    #
    #
    def createRosSubscriber(self, name, datatype, callback):
      if rospy:
        self.initRosNode()
        if self.ros_node:
          self.adaptors[name]=RosAdaptor(name, 'Subscriber')
          if not callback:
            callback=lambda x: eSEAT_Core.onData(self,name, x)
          elif type(callback) == str:
            callback=eval(callback)

          self.adaptors[name].create(name, datatype, callback)

    def createRosServer(self, name, srv_name, srv_type, srv_impl, fname):
      if rospy:
        self.initRosNode()
        if srv_type.find('.') > 0:
            pkgname,srv = srv_type.split('.',1)
        exec_str="import %s.srv as %s" % (pkgname, pkgname)
        exec(exec_str, globals())

        if fname:
            utils.exec_script_file(fname, globals())

        resfunc=eval(srv_type+"Response")
        srv_func=lambda x :  resfunc(eval(srv_impl)(x))
        self.adaptors[name]=rospy.Service(srv_name, eval(srv_type), eval(srv_impl)) 
        
    def createRosClient(self, name, srv_name, srv_type):
      if rospy:
        self.initRosNode()
        if srv_type.find('.') > 0:
            pkgname,srv = srv_type.split('.',1)
        exec_str="import %s.srv as %s" % (pkgname, pkgname)
        exec(exec_str, globals())

        self.adaptors[name]=rospy.ServiceProxy(srv_name, eval(srv_type)) 

    def callRosService(self, name, *args):
        try:
          return self.adaptors[name](*args)
        except:
          print("Error in callRosService %s" % name)
          
    #
    #  Create Adaptor called by SEATML_Parser
    #
    def createAdaptor(self, compname, tag, env=globals()):
        try:
            name = str(tag.get('name'))
            type = tag.get('type')
            self._logger.info(u"!createAdaptor: " + type + ": " + name)

            if type == 'web' :
                dirname = tag.get('document_root')
                if not dirname: dirname = tag.get('dir')
                if dirname:
                    self.createWebAdaptor(name, int(tag.get('port')), compname, tag.get('host'), dirname)
                else:
                    self.createWebAdaptor(name, int(tag.get('port')), compname, tag.get('host'))

            elif type == 'socket' :
                self.createSocketPort(name, tag.get('host'), int(tag.get('port')))
            elif type == 'ros_pub' :
                self.createRosPublisher(name, tag.get('datatype'),int(tag.get('size')))

            elif type == 'ros_sub' :
                fname=tag.get('file')
                if fname:
                  utils.exec_script_file(fname, globals())
                self.createRosSubscriber(name, tag.get('datatype'),tag.get('callback'))
            elif type == 'ros_server' :
                fname=tag.get('file')
                srv_name=tag.get('service')
                srv_type=tag.get('service_type')
                srv_impl=tag.get('impl')
                self.createRosServer(name, srv_name, srv_type, srv_impl, fname)

            elif type == 'ros_client' :
                srv_name=tag.get('service')
                srv_type=tag.get('service_type')
                self.createRosClient(name, srv_name, srv_type)

            else:
                self._logger.warn(u"invalid type: " + type + ": " + name)
                return -1
        except:
            self._logger.error(u"invalid parameters: " + type + ": " + name)
            traceback.print_exc()
            return -1

        return 1

    ###########################
    # Send Data 
    #
    def send(self, name, data, code='utf-8'):
        if isinstance(data, str) :
            self._logger.info("sending message %s (to %s)" % (data, name))
        else:
            self._logger.info("sending message to %s" % (name,))

        dtype = self.adaptortype[name][1]

        if self.adaptortype[name][2]:
            ndata = []
            if type(data) == str :
              for d in data.split(","):
                ndata.append( convertDataType(dtype, d, code) )
              self._data[name].data = ndata
            else:
              self._data[name] = data

        elif dtype == str:
            self._data[name].data = data.encode(code)

        elif dtype == unicode:
            self._logger.info("sending message to %s, %s" % (data,code))
            self._data[name].data = unicode(data)

        elif dtype == int or dtype == float :
            self._data[name].data = dtype(data)

        else:
            try:
                if type(data) == str :
                  self._data[name] = apply(dtype, eval(data))
                else:
                  self._data[name] = data
            except:
                self._logger.error( "ERROR in send: %s %s" % (name , data))

        try:
            self._port[name].write(self._data[name])
        except:
            self._logger.error("Fail to sending message to %s" % (name,))

    ##################################
    #  Event processes 
    #
    #  onData: this method called in comming data
    #
    def onData(self, name, data):
        self.resetTimer()
        try:
            if isinstance(data, str):
                if data :
                    data2 = parseQueryString(data)
                    if data2 :
                        self.processOnDataIn(name, data2)
                    else :
                        self.processResult(name, data)
                        self.processOnDataIn(name, data)
            elif isinstance(data, std_msgs.String):
                self.processResult(name, data.data)
                self.processOnDataIn(name, data.data)
            else:
                self.processOnDataIn(name, data)
        except:
            self._logger.error(traceback.format_exc())

    ############################################
    #   main event process 
    #
    def processResult(self, name, s):
        try:
            s = unicode(s)
        except UnicodeDecodeError:
            s = str(s).encode('string_escape')
            s = unicode(s)

        self._logger.info("got input %s (%s)" % (s, name))
        cmds = None

        if s.count('<?xml') > 0:
            cmds = self.processJuliusResult(name, s)
        else:
            cmds = self.lookupWithDefault(self.currentstate, name, s)

        if not cmds:
            self._logger.info("no command found")
            return False
        #
        #
        cmds.execute(s)
        self.resetTimer()
        return True

    #####################################
    # process for the cyclic execution
    #
    def processExec(self, sname=None, flag=False):
        if sname is None : sname = self.currentstate
        cmds = self.states[sname].onexec

        if not cmds :
            if flag :
                self._logger.info("no command found")
            return False

        cmds.execute('')
        return True

    #####################################
    # process for onActivated
    #
    def processActivated(self, flag=False):
        cmds = self.states['all'].onactivated

        if not cmds :
            if flag :
                self._logger.info("no command found")
            return False
        cmds.execute('')
        return True

    #####################################
    # process for onDectivated
    #
    def processDeactivated(self, sname=None, flag=False):
        if sname is None : sname = self.currentstate
        cmds = self.states[sname].ondeactivated

        if not cmds :
            if flag :
                self._logger.info("no command found")
            return False
        cmds.execute('')
        return True


    ##############################
    # process for timeout
    #
    def processTimeout(self, sname=None, flag=False):
        if sname is None : sname = self.currentstate
        cmds = self.states[sname].ontimeout
        if not cmds :
            if flag :
                self._logger.info("no command found")
            return False
        #
        cmds.execute('')
        self.setTimeout(cmds.timeout)
        self.resetTimer()
        return True

    def resetTimer(self):
        self.last_on_data = time.time()

    def setTimeout(self, sec):
        self._on_timeout = sec

    def isTimeout(self):
       return ( self._on_timeout > 0 and time.time() - self.last_on_data > self._on_timeout )

    #################################
    # Event process for Julius
    #
    def processJuliusResult(self, name, s):
        doc = BeautifulSoup(s)

        for s in doc.findAll('data'):
            rank = int(s['rank'])
            score = float(s['score'])
            text = s['text']
            self._logger.info("#%i: %s (%f)" % (rank, text, score))

            if score < self._scorelimit[0]:
                self._logger.info("[rejected] score under limit")
                continue

            cmds = self.lookupWithDefault(self.currentstate, name, text)
            if cmds:
                setGlobals('julius_result',  {'rank': rank, 'score': score, 'text': text, 'stext': text.split(' ')})
                return cmds
            else:
                self._logger.info("[rejected] no matching phrases")
        return None

    ######################################
    #  Event process for data-in-event
    #
    def initDataIn(self, data):
        setGlobals('seat', self)
        setGlobals('rtc_in_data', data)
        setGlobals('julius_result', None)
        return

    def processOnDataIn(self, name, data):
        self._logger.info("got input from %s" %  (name,))
        cmds = self.lookupWithDefault(self.currentstate, name, "ondata")

        if not cmds:
            self._logger.info("no command found")
            return False

        for cmd in cmds:
            self.initDataIn(data)
            cmd.execute_pre_script()
            cmd.executeEx(data)

        return True

    ####################################################################################
    #  Lookup Registered Commands with default
    #
    def lookupWithDefault(self, state, name, s, flag=True):
        s=s.split(",")[0]
        if flag:
            self._logger.info('looking up...%s: %s' % (name,s,))
        cmds = self.lookupCommand(state, name, s)

        if not cmds:
            cmds = self.lookupCommand(state, 'default', s)

        if not cmds:
            cmds = self.lookupCommand('all', name, s)

        if not cmds:
            cmds = self.lookupCommand('all', 'default', s)

        return cmds

    #
    #  Lookup Registered Commands
    #
    def lookupCommand(self, state, name, s):
        try:
            cmds = self.states[state].matchkey(name, s)
        except:
            cmds = None
        return cmds
        
    #def lookupCommand_old(self, state, name, s):
    #    cmds = []
    #    regkeys = []
    #    try:
    #        k = state+":"+name+":"+s
    #        cmds = self.keys[k]
    #    except KeyError:
    #        try:
    #            regkeys = self.regkeys[state+":"+name]
    #        except KeyError:
    #            return None
    #
    #        for r in regkeys:
    #            if r[0].match(s):
    #                cmds = r[1]
    #                break
    #        return None
    #    return cmds
    #############################
    #  For STATE of eSEAT
    #
    #  Get state infomation
    #
    def getStates(self):
        return self.states

    #
    # set the begining state
    #
    def setStartState(self, name):
        self.startstate = name
        return

    #
    #  Count the number of states 
    #
    def countStates(self):
        return (len(self.states) - 1)

    #
    #  check the named state
    #
    def inStates(self, name):
        return self.states.has_key(name)

    #
    #  append the named state
    #
    #def appendState(self, name):
    #    self.states.extend([name])
    #    return

    #
    #  initilaize the begining state
    #
    def initStartState(self, name):
        self.startstate = None
        if self.states.has_key(name) :
            self.startstate = name
        else:
            self.startstate = self.states.keys()[1]
        self.stateTransfer(self.startstate)
        self._logger.info("current state " + self.currentstate)
    #
    # create the named state
    #
    def create_state(self, name):
        self.items[name] = []
        if not self.states.has_key(name):
            self.states[name] = State(name)
        if self.init_state == None or self.init_state == 'all':
            self.init_state = name
        return 

    ###############################################
    # State Transition for eSEAT
    #
    def stateTransfer(self, newstate):
        try:
            #tasks = self.keys[self.currentstate+":::onexit"]
            tasks = self.states[self.currentstate].onexit
            if tasks:
                tasks.execute()
        except KeyError:
            pass
  
        try:
            self.prev_state=self.currentstate
            self.next_state=newstate
            self.root.event_generate("<<state_transfer>>", when="tail")
        except:
            pass

        self.currentstate = newstate

        try:
            cmds = self.lookupWithDefault(newstate, '', 'ontimeout', False)
            if cmds: self.setTimeout(cmds.timeout)
            #tasks = self.keys[self.currentstate+":::onentry"]
            tasks = self.states[self.currentstate].onentry
            if tasks:
                tasks.execute()
        except KeyError:
            pass


    ##############################
    #  main SEATML loader
    #
    def loadSEATML(self, f):
        self._logger.info("Start loadSEATML:"+f)
        res = self.parser.load(f)
        if res == 1 : 
            self._logger.error("===> SEATML Parser error")
            if self.manager : self.manager.shutdown()
            #sys.exit(1)
        return res

    #
    #  register commands into self.keys
    #
    #def registerCommands(self, key, cmds):
    #    self._logger.info("register key="+key)
    #    self.keys[key] = cmds
    #
    #def appendCommands(self, key, cmds):
    #    self._logger.info(" append key="+key)
    #    self.keys[key].append(cmds) 
    #
    #def registerCommandArray(self, tag, cmds):
    #    if self.keys.keys().count(tag) == 0 :
    #        self.registerCommands(tag, [cmds])
    #    else :
    #        self.appendCommands(tag, cmds)

    ##############################################
    #  Callback function for WebAdaptor
    #     
    def callComet(self):
      res = ""
      return res

    ################################################ 
    #
    #  Sub-process
    #
    def getSubprocessList(self):
        res=[]
        newlst=[]
        for p in self.popen:
            p.poll()
            if p.returncode == None:
                res.append(p.pid)
                newlst.append(p)
        self.popen = newlst
        return res

    def killSubprocess(self, pid=None):
        for p in self.popen:
            p.poll()
            if pid == None or p.pid == pid :
                p.terminate()
        return 
    #
    # Finalize
    #
    def finalizeSEAT(self):
        for a in self.adaptors.itervalues():
            if isinstance(a, SocketAdaptor):
                a.terminate()
                a.join()
            elif isinstance(a, WebSocketServer):
                a.terminate()
        if self.root : self.root.quit()
        return 

###########################################################
#   eSEAT_Gui
#
class eSEAT_Gui:
    def __init__(self):
        self.root = None
        self.gui_items = {}
        self.frames = {}
        self.max_columns = 20
        self.items = {}
        self.inputvar = {}
        self.stext = {}
        self.buttons = {}
        self.labels = {}

    #
    #  check the GUI items
    #
    def hasGUI(self):
        for x in self.items.keys():
            if len(self.items[x]) : return True
        return False

    #########################################
    #  Callback function
    #
    def mkcallback(self, name):
        def callback(event=None):
           self.processResult("gui", name)
        return callback

    def mkinputcallback(self, name):
        def callback(event=None):
           self.processResult("gui", name)
        return callback

    #############################
    #  Create GUI items
    #
    #  Create Frame
    def newFrame(self, name):
        self.frames[name] = Frame(self.root)
        return self.frames[name]

    def setTitle(self, name):
        if self.root : self.root.title(name)

    #  Called by SEATML Parser
    def addButton(self, name, txt, fg, bg, cspan, w):
        self.items[name].append(['button', txt, fg, bg, cspan, w])

    def addEntry(self, name, txt, w, cspan, val=''):
        self.items[name].append(['entry', txt, w, cspan, val])

    def addText(self, name, txt, w, h, cspan, rspan, val=""):
        self.items[name].append(['text', txt, w, h, cspan, rspan, val])

    def addLabel(self, name, txt, fg, bg, cspan):
        self.items[name].append(['label', txt, fg, bg, cspan])

    def addBreak(self, name):
        self.items[name].append(['br', "BR"])

    def addSpace(self, name,n):
        self.items[name].append(['space', n])

    ###################  CREATE GUI ITEMS ####################
    #################  B U T T O N ################### 
    ## Create Button Item
    def createButtonItem(self, frame, sname, name, fg="#000000", bg="#cccccc", cspan=1, w=None):
        #btn = Button(frame, text=name, command=self.mkcallback(name) , bg=bg, fg=fg)
        if w :
            btn = Button(frame, text=name, command=self.mkcallback(name) , bg=bg, fg=fg, width=int(w))
        else:
            btn = Button(frame, text=name, command=self.mkcallback(name) , bg=bg, fg=fg)
        self.buttons[sname+":"+name] = btn
        return [btn, cspan]
    
    def setButtonConfig(self, eid, **cfg):
        try:
            self.buttons[eid].config(**cfg)
        except:
            print ("ERROR in ButtonConfig")
            pass

    #################  L I N E I N P U T ################### 
    ## Create Entry Item
    def createEntryItem(self, frame, sname, name, eid, w, cspan=1, val=''):
        var=StringVar()
        var.set(val.strip())
        self.inputvar[sname+":"+eid] = var
        enty = Entry(frame, textvariable=var, width=int(w))
        self.bind_commands_to_entry(enty, name, eid)

        return [enty, cspan]

    def bind_commands_to_entry(self, entry, name, eid):
        key = name+":gui:"+eid
        #if self.keys[key] :
        if self.states[name].has_rule('gui', eid) :
            self._logger.info("Register Entry callback")
            entry.bind('<Return>', self.mkinputcallback(eid))
        return 

    def getEntry(self, eid):
        try:
            return self.inputvar[eid].get()
        except:
            return ""

    def setEntry(self, eid, txt=""):
        try:
            return self.inputvar[eid].set(txt)
        except:
            return ""

    #################  T E X T A R E A ################### 
    ## Create Text Item
    def createTextItem(self, frame, sname, name, eid, w, h, cspan=1, rspan=1, txt=""):
        stxt = ScrolledText(frame, width=int(w), height=int(h))
        stxt.insert(END, txt.strip())
        stxt.tag_config("csel", background="blue",foreground="white")
        self.stext[sname+":"+eid] = stxt
        key = name+":gui:"+eid

        return [stxt, cspan, rspan]

    def getText(self, eid):
        try:
            return self.stext[eid].get(1.0,END)
        except:
            return ""

    def unsetSelText(self, eid):
        try:
            txt=self.stext[eid]
            rng=txt.tag_ranges("csel")
            txt.tag_remove("csel", rng[0], rng[1])
        except:
            pass

    def getSelText(self, eid):
        try:
            txt=self.stext[eid]
            rng=txt.tag_ranges("csel")
            return txt.get(rng[0], rng[1])
        except:
            return "" 

    def nextSelText(self, eid):
        try:
            txt=self.stext[eid]
            rng=txt.tag_ranges("csel")
            nl=int(txt.index(rng[0]).split(".")[0]) + 1
            if nl > self.getLastIndex(eid) : return nl-1
            self.setSelText(eid, nl)
            return nl 
        except:
            return 0 

    def prevSelText(self, eid):
        try:
            txt=self.stext[eid]
            rng=txt.tag_ranges("csel")
            nl=int(txt.index(rng[0]).split(".")[0]) - 1
            self.setSelText(eid, nl)
            return nl 
        except:
            return 0 

    def getSelTextLine(self, eid):
        try:
            txt=self.stext[eid]
            rng=txt.tag_ranges("csel")
            nl=int(txt.index(rng[0]).split(".")[0])
            return nl 
        except:
            return 0 

    def setSelText(self, eid, n=1):
        try:
            spos='%d.0' % (n)
            epos='%d.0' % (n+1)
            self.unsetSelText(eid)
            self.stext[eid].tag_add("csel", spos, epos)
            self.setCurrentIndex(eid, n)
        except:
            pass

    def getNthLine(self, eid, n=1):
        try:
            spos='%d.0' % (n)
            epos='%d.0' % (n+1)
            return self.stext[eid].get(spos,epos)
        except:
            return ""

    def getLastLine(self, eid, n=1):
        try:
            spos=self.stext[eid].index('end-%dl' % (n))
            epos=self.stext[eid].index('end-%dl' % (n-1))
            return self.stext[eid].get(spos,epos)
        except:
            return ""

    def getLastIndex(self, eid):
        try:
            return int(self.stext[eid].index('end-1c').split('.')[0])
        except:
            return 1

    def getCurrentIndex(self, eid):
        try:
            return int(self.stext[eid].index('insert').split('.')[0])
        except:
            return 0

    def setCurrentIndex(self, eid, n=1):
        try:
            self.stext[eid].mark_set('insert', "%d.0" % (n))
            return n
        except:
            return 0

    def appendText(self, eid, txt=""):
        try:
            val= self.stext[eid].insert(END, txt)
            self.stext[eid].see(END)
            return val
        except:
            return ""

    def insertText(self, eid, pos, txt=""):
        try:
            val= self.stext[eid].insert(pos, txt)
            self.stext[eid].see(pos)
            return val
        except:
            return ""

    def clearText(self, eid):
        try:
            return self.stext[eid].delete(1.0,END)
        except:
            return ""

    #################  L A B E L ################### 
    ## Create Label Item
    def createLabelItem(self, frame, sname, name, fg="#ffffff", bg="#444444", cspan=1):
        if not fg: fg="#ffffff"
        if not bg: bg="#444444"
        lbl = Label(frame, text=name, bg=bg, fg=fg )
        self.labels[sname+":"+name] = lbl
        return [lbl, cspan]

    def setLabelConfig(self, eid, **cfg):
        try:
            self.labels[eid].configure(**cfg)
        except:
            print ("ERROR in setLabelConfig")
            pass

    #########  LAYOUT ITEMS ON A FRAME ############
    ## Layout GUI items
    def packItems(self, name):
        n=self.max_columns
        if self.gui_items[name] :
            i=0
            j=1
            for itm in self.gui_items[name] :
                if ( i % self.max_columns ) == 0:
                    j += 1

                if itm == "BR":
                    j += 1
                    i = 0
                elif itm == "SP":
                    i += 1
                else :
                   if type(itm) == list:
                       itm[0].grid(row=j, column=i, columnspan=itm[1], sticky=W+E)
                       i = i+itm[1] 
                   else :
                       itm.grid(row=j, column=i, sticky=W + E)
                       i = i+1
                   i = i % self.max_columns

    ## Create and layout GUI items
    def createGuiPanel(self, name):
        if name:
           items = self.items[name]
           self.gui_items[name] = []

           for itm in items:
               if itm[0] == 'br':
                   self.gui_items[name].append( "BR" )

               elif itm[0] == 'space':
                   for i in range( int(itm[1] )):
                       self.gui_items[name].append( "SP" )

               elif itm[0] == 'button':
                   self.gui_items[name].append(
                       self.createButtonItem(self.frames[name], name,
                                itm[1], itm[2], itm[3], int(itm[4]), itm[5])
                       )

               elif itm[0] == 'entry':
                   self.gui_items[name].append(
                       self.createEntryItem(self.frames[name], name, name,
                                       itm[1], itm[2], int(itm[3]), itm[4])
                       )

               elif itm[0] == 'text':
                   self.gui_items[name].append(
                       self.createTextItem(self.frames[name],name, name,
                                itm[1], itm[2], itm[3], int(itm[4]), int(itm[5]), itm[6])
                       )

               elif itm[0] == 'label':
                   self.gui_items[name].append(
                       self.createLabelItem(self.frames[name], name,
                                itm[1], itm[2], itm[3], int(itm[4]))
                       )
               else:
                   pass

           self.packItems(name)

        return 0

    ##
    ## Change the view of GUI with state transition
    def stateChanged(self, event=None, *args):
        try:
           self.hideFrame(self.prev_state)
           self.showFrame(self.next_state)
        except:
           pass

    ## Display/Hide GUI Window
    def showFrame(self, name):
        if self.frames[name] :
           self.frames[name].pack()

    def hideFrame(self, name):
        if self.frames[name] :
           self.frames[name].pack_forget()

    #
    #   Event loop for GUI
    #
    def startGuiLoop(self, viewer):
        try:
          self.root = Tk()
        except:
          return 1

        if self.hasGUI() : 
            for st in self.states:
                self.newFrame(st)
                self.createGuiPanel(st)

            self.showFrame(self.init_state)
            self.frames[self.init_state].pack()

            self.root.bind("<<state_transfer>>", self.stateChanged)
            self.setTitle(self.getInstanceName())

        if viewer:
            viewer.createViewer(self.root, self.hasGUI())

        self.root.mainloop()
        return 0
