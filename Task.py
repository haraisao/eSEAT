#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Task and TaskGroup for eSEAT (Extened Simple Event Action Transfer)

Copyright (C) 2017
    Isao Hara
    Robot Innovation Rearch Center,
    National Institute of Advanced Industrial Science and Technology (AIST), Japan
    All rights reserved.
  Licensed under the MIT License (MIT)
  http://www.opensource.org/licenses/MIT
'''

############### import libraries
import sys
import os
import subprocess

import utils
import re
import traceback

from eSEAT_Core import getGlobals, setGlobals
'''
State:
  - name(string), rules([TaskGroup,]), onentry(TaskGroup), onexit(TaskGroup)

TaskGroup: <-- <rule>
  - keys([string,]), patterns([reg,]), taskseq([Task,])

 Task: <- <message>, <shell>, <script>, <log>, <statetransition>
   +- TaskMessage: <message>
        - sendto, encode, input
   +- TaskShell: <shell>
        - sendto
   +- TaskScript: <sciptt>
        - execfile, sendto
   +- TaskLog: <log>
        - 
   +- TaskStateTransision: <statetransition>
        - func[push, pop]

'''
###########################################
#
#  Class Task for eSEAT
#
class Task():
    def __init__(self, rtc):
        self.seat = rtc
        self._logger = rtc._logger
        
    def execute(self,data):
        return True

class TaskMessage(Task):
    def __init__(self, rtc, sndto, data, encode, input_id):
        Task.__init__(self, rtc)
        self.sendto=sndto
        self.data = data
        self.encoding = encode
        self.input_id=input_id
        
        return

    def execute(self, s):
        data = self.data
        try:
            ad = self.seat.adaptors[self.sendto]
            if self.input_id :
                if self.seat.inputvar.has_key(self.input_id) :
                    data = self.seat.inputvar[self.input_id].get()
                elif self.seat.stext.has_key(self.input_id) :
                    data = self.seat.getLastLine(self.input_id, 1)
            #
            #  Call 'send' method of Adaptor
            #
            if not self.encoding :
                ad.send(self.sendto, data)
            else :
                ad.send(self.sendto, data, self.encoding)
            return True

        except KeyError:
            if self.sendto :
                self._logger.error("no such adaptor:" + self.sendto)
            else :
                self._logger.error("no such adaptor: None")
            return False

class TaskShell(Task):
    def __init__(self, rtc, sendto, data):
        Task.__init__(self, rtc)
        self.sendto = sendto
        self.data = data
        return

    def execute(self, data):
        #
        # execute shell command with subprocess
        res = subprocess.Popen(self.data, shell=True)
        self.popen.append(res)
        #
        #  Call 'send' method of Adaptor
        try:
            ad = self.seat.adaptors[self.sendto]
            ad.send(self.sendto, res)
        except KeyError:
            if name :
               self._logger.error("no such adaptor:" + self.sendto)
            else:
               self._logger.error("no such adaptor: None")

class TaskScript(Task):
    def __init__(self, rtc, sendto, data, fname):
        Task.__init__(self, rtc)
        self.sendto = sendto
        self.data = data
        self.fname = fname
        return

    def execute(self, data):
        setGlobals('rtc_result', None)
        setGlobals('rtc_in_data', data)
        setGlobals('web_in_data', data)
        #
        #   execute script or script file
        if self.fname :
            ffname = utils.findfile(self.fname)
            if ffname :
                exec_script_file(ffname, getGlobals())
        try:
            if self.data :
                exec(self.data, getGlobals())
        except:
            self._logger.error("Error:" + self.data)
            print traceback.format_exc()
            return False
        # 
        #  Call 'send' method of Adaptor to send the result...
        rtc_result = getGlobals()['rtc_result'] 
        if rtc_result != None :
            try:
                ad = self.seat.adaptors[self.sendto]
                ad.send(self.sendto, rtc_result)
            except KeyError:
                if self.sendto :
                   self._logger.error("no such adaptor:" + self.sendto)
                else:
                   self._logger.error("no such adaptor: None")
                return False
        return True

class TaskLog(Task):
    def __init__(self, rtc, data):
        Task.__init__(self, rtc)
        self.info = data
        return
    def execute(self, data):
        self._logger.info(self.info)
        return True

class TaskStatetransition(Task):
    def __init__(self, rtc, func, data):
        Task.__init__(self, rtc)
        self.func = func
        self.data = data
        return

    def execute(self, data):
        try:
            if (self.func == "push"):
                self.seat.statestack.append(self.seat.currentstate)
                self.seat.stateTransfer(self.data)

            elif (self.func == "pop"):
                if self.seat.statestack.__len__() == 0:
                    self._logger.warn("state buffer is empty")
                    return False
                self.seat.stateTransfer(self.seat.statestack.pop())
            else:
                self._logger.info("state transition from "+self.seat.currentstate+" to "+self.data)
                self.seat.stateTransfer(self.data)
            return True
        except:
            return False


#############################
#  TaskGroup Class for eSEAT:
#      TaskGroup neary equal State....
#
class TaskGroup():
    def __init__(self):
        self.taskseq=[]
        self.keys=[]
        self.patterns=[]
        self.timeout = -1

    def execute(self, data=None):
        for cmd in self.taskseq:
            cmd.execute(data)

    def executeEx(self, data=None):
        for cmd in self.taskseq:
            if isinstance(cmd, TaskMessage):
                cmd.encoding = None
            cmd.execute(data)

    def addTask(self, task):
        self.taskseq.append(task)

    def clearTasks(self):
        self.taskseq=[]

    def addPattern(self, pat):
        self.patterns.append(re.compile(pat))

    def addKey(self, key):
        self.keys.append(key)

    def match(self, msg):
        for x in self.keys:
            if x == msg : return True
        for x in self.patterns:
            if x.match(msg) : return True
        return False
    
#####################
#  State
class State():
    def __init__(self, name):
        self.name = name
        self.onentry = None
        self.onexit = None
        self.rules = []
