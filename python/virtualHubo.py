#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__author__ = 'Daniel M. Lofaro'
__license__ = 'GPLv3 license'

import hubo_ach as ha
import ach
import time
import numpy as np
# from ctypes import *

from optparse import OptionParser
# import select

# from openravepy import *
from openravepy import raveLog
from numpy import pi
# import time
# import sys
import openhubo

skip = 100
skipi = 0
skiptemp = 0.0
hubo_timestep = 0.0005
FLAG_DRC = False


class StatusLogger:
    """Simple and efficient status updater for the main loop"""
    def __init__(self,skipcount=100,init_time=0.0):
        self.t_last=init_time
        self.skip=skipcount
        self.count=0
        self.rate=0.0

    def tick(self):
        self.count+=1
        if self.count>=self.skip:
            self.show()

    def show(self):
        ideal_time = ha.HUBO_LOOP_PERIOD*self.count
        t=time.time()
        actual_time = t-self.t_last
        virtualHuboLog('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}% T= {:.6f}'.format(ideal_time,actual_time,ideal_time/actual_time*100,openhubo.TIMESTEP))  #	
        #virtualHuboLog('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}%'.format(ideal_time,actual_time,ideal_time/actual_time*100))
        self.t_last=t
        self.count=0

class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        global skip
        global skipi
        global skiptemp
        skiptemp = skiptemp + (time.time() - self.tstart)
        if (skipi < skip):
            skipi = skipi + 1
        else:
            skiptemp = skiptemp/skip
            if self.name:
                print '[%s]' % self.name,
           # print 'Elapsed: %s' % (time.time() - self.tstart)
            print 'Elapsed: ',skiptemp,' sec : ', (ha.HUBO_LOOP_PERIOD/skiptemp * 100.0),' percent'
            skipi = 0

def sim2state(robot,state):
    global FLAG_DRC
    pose=robot.GetDOFValues()
    # Get current state from simulation
    if(FLAG_DRC):
      state.joint[ha.RWR].pos = pose[ind('RWR')]
      state.joint[ha.LWR].pos = pose[ind('LWR')]

    state.joint[ha.RSP].pos = pose[ind('RSP')]
    state.joint[ha.RSR].pos = pose[ind('RSR')]
    state.joint[ha.RSY].pos = pose[ind('RSY')]
    state.joint[ha.REB].pos = pose[ind('REP')]
    state.joint[ha.RWY].pos = pose[ind('RWY')]
    state.joint[ha.RWP].pos = pose[ind('RWP')]

    state.joint[ha.LSP].pos = pose[ind('LSP')]
    state.joint[ha.LSR].pos = pose[ind('LSR')]
    state.joint[ha.LSY].pos = pose[ind('LSY')]
    state.joint[ha.LEB].pos = pose[ind('LEP')]
    state.joint[ha.LWY].pos = pose[ind('LWY')]
    state.joint[ha.LWP].pos = pose[ind('LWP')]

    state.joint[ha.WST].pos = pose[ind('HPY')]

    state.joint[ha.RHY].pos = pose[ind('RHY')]
    state.joint[ha.RHR].pos = pose[ind('RHR')]
    state.joint[ha.RHP].pos = pose[ind('RHP')]
    state.joint[ha.RKN].pos = pose[ind('RKP')]
    state.joint[ha.RAP].pos = pose[ind('RAP')]
    state.joint[ha.RAR].pos = pose[ind('RAR')]

    state.joint[ha.LHY].pos = pose[ind('LHY')]
    state.joint[ha.LHR].pos = pose[ind('LHR')]
    state.joint[ha.LHP].pos = pose[ind('LHP')]
    state.joint[ha.LKN].pos = pose[ind('LKP')]
    state.joint[ha.LAP].pos = pose[ind('LAP')]
    state.joint[ha.LAR].pos = pose[ind('LAR')]

    state.joint[ha.RF1].pos = pose[ind('RF1')]
    state.joint[ha.RF2].pos = pose[ind('RF2')]
    state.joint[ha.LF1].pos = pose[ind('LF1')]

    return pose

def pos2robot(robot, state):
    global FLAG_DRC
    # Sets the CMD reference to the robot
    pose=robot.GetDOFValues() # gets the current state
    if(FLAG_DRC):
      pose[ind('RWR')] = state.joint[ha.RWR].pos
      pose[ind('LWR')] = state.joint[ha.LWR].pos

    pose[ind('RSP')] = state.joint[ha.RSP].pos
    pose[ind('RSR')] = state.joint[ha.RSR].pos
    pose[ind('RSY')] = state.joint[ha.RSY].pos
    pose[ind('REP')] = state.joint[ha.REB].pos
    pose[ind('RWY')] = state.joint[ha.RWY].pos
    pose[ind('RWP')] = state.joint[ha.RWP].pos

    pose[ind('LSP')] = state.joint[ha.LSP].pos
    pose[ind('LSR')] = state.joint[ha.LSR].pos
    pose[ind('LSY')] = state.joint[ha.LSY].pos
    pose[ind('LEP')] = state.joint[ha.LEB].pos
    pose[ind('LWY')] = state.joint[ha.LWY].pos
    pose[ind('LWP')] = state.joint[ha.LWP].pos

    pose[ind('HPY')] = state.joint[ha.WST].pos

    pose[ind('RHY')] = state.joint[ha.RHY].pos
    pose[ind('RHR')] = state.joint[ha.RHR].pos
    pose[ind('RHP')] = state.joint[ha.RHP].pos
    pose[ind('RKP')] = state.joint[ha.RKN].pos
    pose[ind('RAP')] = state.joint[ha.RAP].pos
    pose[ind('RAR')] = state.joint[ha.RAR].pos

    pose[ind('LHY')] = state.joint[ha.LHY].pos
    pose[ind('LHR')] = state.joint[ha.LHR].pos
    pose[ind('LHP')] = state.joint[ha.LHP].pos
    pose[ind('LKP')] = state.joint[ha.LKN].pos
    pose[ind('LAP')] = state.joint[ha.LAP].pos
    pose[ind('LAR')] = state.joint[ha.LAR].pos

    pose[ind('RF1')] = state.joint[ha.RF1].pos
    pose[ind('RF2')] = state.joint[ha.RF2].pos
    pose[ind('LF1')] = state.joint[ha.LF1].pos

    return pose

def ref2robot(robot, state):
    global FLAG_DRC
    # Sets the CMD reference to the robot
    pose=robot.GetDOFValues() # gets the current state
    if(FLAG_DRC):
      pose[ind('RWR')] = state.joint[ha.RWR].ref
      pose[ind('LWR')] = state.joint[ha.LWR].ref

    pose[ind('RSP')] = state.joint[ha.RSP].ref
    pose[ind('RSR')] = state.joint[ha.RSR].ref
    pose[ind('RSY')] = state.joint[ha.RSY].ref
    pose[ind('REP')] = state.joint[ha.REB].ref
    pose[ind('RWY')] = state.joint[ha.RWY].ref
    pose[ind('RWP')] = state.joint[ha.RWP].ref

    pose[ind('LSP')] = state.joint[ha.LSP].ref
    pose[ind('LSR')] = state.joint[ha.LSR].ref
    pose[ind('LSY')] = state.joint[ha.LSY].ref
    pose[ind('LEP')] = state.joint[ha.LEB].ref
    pose[ind('LWY')] = state.joint[ha.LWY].ref
    pose[ind('LWP')] = state.joint[ha.LWP].ref

    pose[ind('HPY')] = state.joint[ha.WST].ref

    pose[ind('RHY')] = state.joint[ha.RHY].ref
    pose[ind('RHR')] = state.joint[ha.RHR].ref
    pose[ind('RHP')] = state.joint[ha.RHP].ref
    pose[ind('RKP')] = state.joint[ha.RKN].ref
    pose[ind('RAP')] = state.joint[ha.RAP].ref
    pose[ind('RAR')] = state.joint[ha.RAR].ref

    pose[ind('LHY')] = state.joint[ha.LHY].ref
    pose[ind('LHR')] = state.joint[ha.LHR].ref
    pose[ind('LHP')] = state.joint[ha.LHP].ref
    pose[ind('LKP')] = state.joint[ha.LKN].ref
    pose[ind('LAP')] = state.joint[ha.LAP].ref
    pose[ind('LAR')] = state.joint[ha.LAR].ref

    pose[ind('RF1')] = state.joint[ha.RF1].ref
    pose[ind('RF2')] = state.joint[ha.RF2].ref
    pose[ind('LF1')] = state.joint[ha.LF1].ref

    return pose

def virtualHuboLog(string,level=4):
    raveLog('[virtualHubo.py] '+string,level)

if __name__=='__main__':
    global hubo_timestep
    global FLAG_DRC


    parser = OptionParser()

    (options, args) = parser.parse_args()
    (env,options)=openhubo.setup('qtcoin',True)
    print 'all args = ', args
    try:
        flag = args[0]
    except:
        flag = 'unspecified'
    try:
        simtimeFlag = args[1]
    except:
        simtimeFlag = 'unspecified'

    for arg in args:
      print 'arg = ', arg
      if arg == 'drc':
        options.robotfile = '/etc/hubo-ach/sim/drchubo/drchubo-v3/robots/drchubo-v3.robot.xml'
        FLAG_DRC = True
        hubo_timestep = 0.001



    print 'dan: ',options.robotfile
    env.SetDebugLevel(4)
    time.sleep(.25)

    try:
        oh_version=openhubo.__version__
    except AttributeError:
        oh_version='old'
    virtualHuboLog("Detected OpenHubo version {}".format(oh_version))

    # Set up simulation options based on command line args
    options.ghost=True

    if flag == 'nophysics':
        options.physics=False
        options.stop=False
        openhubo.TIMESTEP=ha.HUBO_LOOP_PERIOD
        print 'No Dynamics mode'
    else:
        options.physics=True
        options.stop=True

    #Enable simtime if required, or if force by physics
    options.simtime = (simtimeFlag == 'simtime') or options.physics

    # Detect Load robot and scene based on openhubo version
    if oh_version=='0.8.0':
        [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)
    elif oh_version=='0.7.0':
        [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options.robotfile,options.scenefile,options.stop, options.physics, options.ghost)
    else:
        [robot,ctrl,ind,ghost,recorder]=openhubo.load(env,options.robotfile,options.scenefile,options.stop, options.physics, options.ghost)

    # Setup ACH channels to interface with hubo
    s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
    s.flush()
    state = ha.HUBO_STATE()

    ts = ach.Channel(ha.HUBO_CHAN_VIRTUAL_TO_SIM_NAME)
    ts.flush()
    sim = ha.HUBO_VIRTUAL()

    fs = ach.Channel(ha.HUBO_CHAN_VIRTUAL_FROM_SIM_NAME)
    fs.flush()

    virtualHuboLog('Starting Simulation...',3)

    fs.put(sim)
    statuslogger=StatusLogger(100,time.time())
    while True:
        openhubo.TIMESTEP = hubo_timestep
        statuslogger.tick()

        if options.simtime:
            [status, framesizes] = ts.get(sim, wait=True, last=False)

        [status, framesizes] = s.get(state, wait=False, last=True)

        #Extract pose information from ACH Channels
        ref_pose = ref2robot(robot, state)
        body_pose = pos2robot(robot, state)

        if not options.physics:
            # Simulation is a "viewer", shows current pos and ref
            ghost.SetDOFValues(ref_pose)
            robot.SetDOFValues(body_pose)
        else:
            # Simulation functions as robot "emulator", pass ref
            # channel to controller
            ctrl.SetDesired(ref_pose)

        if options.simtime:
            #Update "official" time based on simulation steps
            N = np.ceil(ha.HUBO_LOOP_PERIOD/openhubo.TIMESTEP)
            for x in xrange(int(N)):
                env.StepSimulation(openhubo.TIMESTEP)  # this is in seconds
            sim.time += N*openhubo.TIMESTEP

            if options.physics:
                pose = sim2state(robot,state)

            # put the current state
            s.put(state)
            fs.put(sim)
        else:
            env.StepSimulation(openhubo.TIMESTEP)  # this is in seconds

        time.sleep(0.001)  # sleep to allow for keyboard input

