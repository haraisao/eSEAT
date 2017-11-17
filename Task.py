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
from eSEAT_Core import getGlobals, setGlobals
import utils

###########################################
#
#  Class Task for eSEAT
#
class Task():
    def __init__(self, rtc, typ, cmd):
        self.seat = rtc
        self.commads=[cmd]
        self.type = typ
        self._logger = rtc._logger

    def addCommand(self, cmd):
        self.commands.append(cmd)

    def clearCommand(self):
        self.commands=[]
        
    def execute(self):
        if self.type == 'message': return self.applyMessage()
        elif self.type == 'shell': return self.applyShell()
        elif self.type == 'script': return self.applyScript()
        else self.type == 'log': return self.applyLog()
        else self.type == 'transition': return self.applyTransition()
        return False

    ############ T A G Operations
    #
    #  Execute <message>
    #
    def applyMessage(self):
        name = self.commands[0]
        data = self.commands[1]
        encoding = self.commands[2]
        input_id = self.commands[3]

        try:
            ad = self.seat.adaptors[name]
            if input_id :
                if self.seat.inputvar.has_key(input_id) :
                    data = self.seat.inputvar[input_id].get()
                elif self.seat.stext.has_key(input_id) :
                    data = self.seat.getLastLine(input_id, 1)
            #
            #  Call 'send' method of Adaptor
            #
            if not encoding :
                ad.send(name, data)
            else :
                ad.send(name, data, encoding)
            return True

        except KeyError:
            if name :
                self._logger.error("no such adaptor:" + name)
            else :
                self._logger.error("no such adaptor: None")
            return False
    #
    #  Execute <statetransition>
    #
    def applyTransition(self):
        try:
            func = self.commands[0]
            data = self.commands[1]

            if (func == "push"):
                self.seat.statestack.append(self.currentstate)
                self.seat.stateTransfer(data)

            elif (func == "pop"):
                if self.seat.statestack.__len__() == 0:
                    self._logger.warn("state buffer is empty")
                    return False
                self.seat.stateTransfer(self.statestack.pop())
            else:
                self._logger.info("state transition from "+self.currentstate+" to "+data)
                self.seat.stateTransfer(data)
            return True
        except:
            return False
    #
    #  Execute <log>
    #
    def applyLog(self):
        self._logger.info(' '.join(self.commands))
        return True
    #
    #  Execute <shell>
    #
    def applyShell(self):
        name = self.commands[0]
        data = self.commands[1]
        #
        # execute shell command with subprocess
        res = subprocess.Popen(data, shell=True)
        self.popen.append(res)
        #
        #  Call 'send' method of Adaptor
        try:
            ad = self.seat.adaptors[name]
            ad.send(name, res)
        except KeyError:
            if name :
               self._logger.error("no such adaptor:" + name)
            else:
               self._logger.error("no such adaptor: None")
    #
    #  Execute <script>
    #
    def applyScript(self):
        name,data,fname, indata = self.commands

        setGlobals('rtc_result', None)
        setGlobals('rtc_in_data', indata)
        setGlobals('web_in_data', indata)
        #
        #   execute script or script file
        if fname :
            ffname = utils.findfile(fname)
            if ffname :
                exec_script_file(ffname, getGlobals())
        try:
            if data :
                exec(data, getGlobals())
        except:
            self._logger.error("Error:" + data)
            return False
        # 
        #  Call 'send' method of Adaptor to send the result...
        rtc_result = getGlobals()['rtc_result'] 
        if rtc_result == None :
            pass
        else:
            try:
                ad = self.seat.adaptors[name]
                ad.send(name, rtc_result)
            except KeyError:
                if name :
                   self._logger.error("no such adaptor:" + name)
                else:
                   self._logger.error("no such adaptor: None")
                return False

        return True

#############################
#  TaskGroup Class for eSEAT:
#      TaskGroup neary equal State....
#
class TaskGroup():
    def __init__(self, cmdlist=[]):
        self.tasklist=cmdlist

    def execute(self):
        for self.list as cmd:
            cmd.execute()

    def addTask(self, task):
        self.tasklist.append(task)

    def clearTasks(self):
        self.tasklist=[]

    
#####################
#  State
class State():
    def __init__(self, name):
        self.name = name
        self.onentry = None
        self.onexit = None
        self.tasks = None
