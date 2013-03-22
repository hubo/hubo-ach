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
from ctypes import *

from optparse import OptionParser
import select

from openravepy import *
from numpy import *
import time
import sys
from servo import *
import openhubo 

skip = 100
skipi = 0
skiptemp = 0.0


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
            skiptemp = skiptemp/100.0
            if self.name:
                print '[%s]' % self.name,
           # print 'Elapsed: %s' % (time.time() - self.tstart)
            print 'Elapsed: ',skiptemp,' sec : ', (ha.HUBO_LOOP_PERIOD/skiptemp * 100.0),' percent'
            skipi = 0

def sim2state(robot,state):

        pose=robot.GetDOFValues() # gets the current state
        # Get current state from simulation 
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

        return pose

def pos2robot(robot, state):
    # Sets the CMD reference to the robot
        pose=robot.GetDOFValues() # gets the current state
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

        return pose

def ref2robot(robot, state):
    # Sets the CMD reference to the robot
        pose=robot.GetDOFValues() # gets the current state
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

        return pose



if __name__=='__main__':

    parser = OptionParser()
#    parser.add_option("-n", "--file", dest="filename",
#        help="write report to FILE", metavar="FILE")
#    parser.add_option("-q", "--quiet",
#        action="store_false", dest="verbose", default=True,
#        help="don't print status messages to stdout")

    (options, args) = parser.parse_args()


#    print 'options: ', options
#    print 'args: ', args[0], ' : num args = ',len(args)

    try:
        flag = args[0]
    except:
        flag = -1


    (env,options)=openhubo.setup('qtcoin',True)
    env.SetDebugLevel(4)
    time.sleep(.25)

    if( 'nodynamics' == flag ):
         print 'No Dynamic mode'
         [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,options.scenefile,True, None, True)  # this will disable physics
    else:
         [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,options.scenefile,True)
         ctrl.SendCommand('set radians ')
    time.sleep(.5)
    env.StartSimulation(openhubo.TIMESTEP)
    time.sleep(.5)
   
    #Change the pose to lift the elbows and send


    # Hubo-Ach Start and setup:
    s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
    s.flush()
    state = ha.HUBO_STATE()
    
    ts = ach.Channel(ha.HUBO_CHAN_VIRTUAL_TO_SIM_NAME)
    ts.flush()
    sim = ha.HUBO_VIRTUAL()
    
    fs = ach.Channel(ha.HUBO_CHAN_VIRTUAL_FROM_SIM_NAME)
    fs.flush()

# start edit here
#    print('Press ENTER to start sim')
#    tmp = raw_input()
#    print tmp

    print('Starting Sim')

    if( 'dynamics' == flag ):
        env.StopSimulation()

    fs.put(sim) 
    while(1):
      with Timer('Get_Pose'):
        if( 'dynamics' == flag ):
            [statuss, framesizes] = ts.get(sim, wait=True, last=False)

        [statuss, framesizes] = s.get(state, wait=False, last=True)

        if( 'nodynamics' == flag ):
            pose = ref2robot(ref, state)
            ref.SetDOFValues(pose)
            pose = pos2robot(robot, state)
            robot.SetDOFValues(pose)
#            pose = ref2robot(robot, state)
        else:
            # Set Reference from simulation
            pose = ref2robot(robot, state)
            ctrl.SetDesired(pose)   # sends to robot



    # this will step the simulation  note: i can run env step in a loop if nothign else changes

        if( 'dynamics' == flag ):
            N = numpy.ceil(ha.HUBO_LOOP_PERIOD/openhubo.TIMESTEP)
            T = 1/N*ha.HUBO_LOOP_PERIOD
    #        print 'openhubo.TIMESTEP = ',openhubo.TIMESTEP, ' : N = ', N, ' : T = ', T
            for x in range(0,int(N)):
                env.StepSimulation(openhubo.TIMESTEP)  # this is in seconds
                sim.time = sim.time + openhubo.TIMESTEP
            s.put(state)
            fs.put(sim) 
            pose = sim2state(robot,state)
        else:
            env.StepSimulation(openhubo.TIMESTEP)  # this is in seconds
# put the current state

        time.sleep(0.001)  # sleep to allow for keyboard input


# end here
    openhubo.pause(2)

    #Hack to get hand 
    if robot.GetName() == 'rlhuboplus' or robot.GetName() == 'huboplus':
        ctrl.SendCommand('openloop '+' '.join(['{}'.format(x) for x in range(42,57)]))
        for i in range(42,57):
            pose[i]=pi/2
        ctrl.SetDesired(pose)
        openhubo.pause(2)

        pose[42:57]=0
        ctrl.SetDesired(pose)
