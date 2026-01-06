from klampt import *
from klampt.math import vectorops,so3,se3
from common import *
import random
import math
import numpy as np
import scipy as sp
from scipy.optimize import leastsq
import pylab as plt

event = 'C'

#difficulty 
#difficulty = 'easy'
difficulty = 'medium'
#difficulty = 'hard'

omniscient_sensor = False

random_seed = 12345
#random_seed = random.seed()
BALL = (1,0,0)
# Time to collect position data points to fit the curve
WAITTIME = 5
BALL_HITTIME=3
RIGHT_TARGET = 172
LEFT_TARGET  = 130 + 23 # Width of ball
SWING_TIME =0.64

RIGHT_INTERVALS=[1.90,2.19,2.68]
LEFT_INTERVALS=[2.10,2.40,2.68]

verbose = True


################ STATE ESTIMATION #####################

class MyObjectStateEstimator:
    """Your own state estimator that will provide a state estimate given
    CameraColorDetectorOutput readings."""
    def __init__(self):
        self.reset()
        self.Tsensor = None
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        self.w,self.h = 320,240
        self.fov = 90
        self.dmax = 5
        if event == 'A':
            #at goal post, pointing a bit up and to the left
            self.Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        elif event == 'B':
            #on ground near robot, pointing up and slightly to the left
            self.Tsensor = (so3.mul(so3.rotation([1,0,0],-0.10),so3.mul(so3.rotation([0,-1,0],math.radians(90)),cameraRot)),[-1.5,-0.5,0.25])
            self.w = 640
            self.h = 480
            self.dmax = 10
        else:
            #on ground near robot, pointing to the right
            self.Tsensor = (cameraRot,[-1.5,-0.5,0.25])
        self.dt = 0.02
        return
    def reset(self):
        pass
    def update(self,observation):
        """Produces an updated MultiObjectStateEstimate given a CameraColorDetectorOutput
        sensor reading."""
        #TODO
        return MultiObjectStateEstimate([])


################### CONTROLLER ########################

class MyController:
    """Attributes:
    - world: the WorldModel instance used for planning.
    - objectStateEstimator: a StateEstimator instance, which you may set up.
    - state: a string indicating the state of the state machine. TODO:
      decide what states you want in your state machine and how you want
      them to be named.
    """
    
    global block1,block2,block3
    
    global first,get_width, move_after_hit
   
    block1=[]
    block2=[]
    block3=[] 
    #width=[]
    
    first = True
    # Boolean to collect width only once
    get_width=True
    move_after_hit = False
  
    
    #print " First " , first
        
    def __init__(self,world,robotController):
        self.world = world
        self.objectStateEstimator = None
        self.state = None
        self.robotController = robotController
        self.reset(robotController)
        self.width = []
        self.block1Fit=None
        self.block2Fit=None
        self.block3Fit=None
        self.hit_time = 0
        self.time = 0
        self.dt = 0
        self.ballPresent = False
        self.hitLeft = False
        
        
    def reset(self,robotController):
        # Caled only once on initialization 
        """Called on initialization, and when the simulator is reset.
        TODO: You may wish to fill this in with custom initialization code.
        """
        self.objectStateEstimator = MyObjectStateEstimator()
        self.objectEstimates = None
        self.state = 'start'
        #TODO: you may want to do more here to set up your
        #state machine and other initial settings of your controller.
        #The 'waiting' state is just a placeholder and you are free to
        #change it as you see fit.
        self.qdes = robotController.getCommandedConfig()
        self.initVis()
        
        block1=[]
        block2=[]
        block3=[] 

        pass

    def myPlayerLogic(self,
                      dt,
                      sensorReadings,
                      objectStateEstimate,
                      robotController):
        """
        TODO: fill this out to updates the robot's low level controller
        in response to a new time step.  This is allowed to set any
        attributes of MyController that you wish, such as self.state.
        
        Arguments:
        - dt: the simulation time elapsed since the last call
        - sensorReadings: the sensor readings given on the current time step.
          this will be a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading
          coming from the omniscient object sensor.  You will not need to
          use raw sensor data directly, if you have a working state estimator.
        - objectStateEstimate: a MultiObjectStateEstimate class (see
          stateestimation.py) produced by the state estimator.
        - robotController: a SimRobotController instance giving access
          to the robot's low-level controller.  You can call any of the
          methods.  At the end of this call, you can either compute some
          PID command via robotController.setPIDCommand(), or compute a
          trajectory to execute via robotController.set/addMilestone().
          (if you are into masochism you can use robotController.setTorque())
        """
        #these are pulled out here for your convenience
        # No need to use this
        qcmd = robotController.getCommandedConfig()
        vcmd = robotController.getCommandedVelocity()
        qsns = robotController.getSensedConfig()
        vsns = robotController.getSensedVelocity()

        if self.state == 'start':
            self.moveRobotToInitialPosition()
        elif self.state == 'waiting':
            self.checkReadyToHit()
        elif self.state == 'hitBall':
            self.hitBall()
        else:
            pass
        return
    
    def moveRobotToInitialPosition(self):
        global move_after_hit
        if(self.hitLeft):
            self.qdes = [0.0, 1.47, -1.08, 1.187, 1.5708, 0.0, 0.0]
        else:
            self.qdes = [0.0, 1.47, -0.92 , 1.45, 1.5708, 0.0, 0.0]
        self.robotController.addMilestone(self.qdes)
        
        if (self.robotController.getCommandedConfig()==self.qdes):
            if(self.time > WAITTIME):                                                                                                                         
                if (not move_after_hit):
                    self.state = 'waiting'
                elif (move_after_hit and self.time - self.hit_time >=4):
                    move_after_hit = False
                else:
                    pass
                    
     
    def loop(self,dt,robotController,sensorReadings):
        """Called every control loop (every dt seconds).
        Input:
        - dt: the simulation time elapsed since the last call
        - robotController: a SimRobotController instance. Use this to get
          sensor data, like the commanded and sensed configurations.
        - sensorReadings: a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading coming
          from the omniscient object sensor.
        
        #ASK setPIDCommand
        Output: None.  However, you should produce a command sent to
          robotController, e.g., robotController.setPIDCommand(qdesired).

        """
        self.time +=dt
        self.dt = dt

        multiObjectStateEstimate = None
            #(1,0,0) : Red (ball)
            #(1,0.5,0) : Orange (1st block)
            #(1,1,0) : Yellow (second block)
            #(0.5,1,0) : green (last block)
        global get_width
        if self.objectStateEstimator and 'blobdetector' in sensorReadings:
            for keys, values in sensorReadings.items():
                for blob in values.blobs:
                    if(blob.color == BALL):
                        self.ballPresent = True
                        
            multiObjectStateEstimate = self.objectStateEstimator.update(sensorReadings['blobdetector'])
            self.objectEstimates = multiObjectStateEstimate
            
            # Check if sifficient Data has been collected for curve fit
            if(self.time <=WAITTIME):
                for keys, values in sensorReadings.items():
                    for blob in values.blobs :
                        if(blob.color==(1,0.5,0)):
                            block1.append(blob.x)
                            
                            if (get_width):
                                self.width.append(blob.w)
                        elif(blob.color==(1,1,0)):
                            block2.append(blob.x)
                            if (get_width):
                                self.width.append(blob.w)
                        elif (blob.color ==(0.5,1,0)):
                            block3.append(blob.x)
                            if (get_width):
                                self.width.append(blob.w)
                        if (blob.color ==BALL):
                            if (get_width):
                                self.width.append(blob.w)
                if (len(self.width)>=4):
                    get_width = False
            else:
                global first
                if(first):
                    # Initial x position data is an outlier so remove it  for all blocks
                    block1.pop(0)
                    block2.pop(0)
                    block3.pop(0) 
                    
                    # Perform sineFit on the x position data for all blocks
                    self.block1Fit = self.sinFit(block1)
                    self.block2Fit = self.sinFit(block2)
                    self.block3Fit = self.sinFit(block3)
                    
                    first = False

        # Because my sensor works, I dont test for the omniscient case   
        if 'omniscient' in sensorReadings:
             # Values in OmniscientObjectOutput
            omniscientObjectState = OmniscientStateEstimator().update(sensorReadings['omniscient'])
            
            #omniscientObjectStateEstimate is now a MultiObjectStateEstimate (see common.py)
            multiObjectStateEstimate  = omniscientObjectState
            #uncomment if you want to see traces
            #self.objectEstimates = multiObjectStateEstimate
            
            # vals = sensorReadings['omniscient']
            # for name in vals.names:
            #     if (name == (1,1,0,1)):
            #         for pos in vals.positions:
            #             print pos
            
        self.myPlayerLogic(dt,
                           sensorReadings,multiObjectStateEstimate,
                           robotController)

        self.updateVis()
        return
    
    def hitBall(self):
        # Keeps track of movig robot arm after hitting 
        global move_after_hit
        # Checks which direction to hit and assigns config accordingly
        if(self.hitLeft):
            self.qdes = [0.0, 2.3, -0.92 , 1.45, 1.5708, 0.0, 0.0]
            self.robotController.addMilestone(self.qdes)
        else:
            self.qdes = [0.0, 1.74, -1.08, 1.187, 1.5708, 0.0, 0.0]
            dt = 0.099
            self.robotController.setLinear(self.qdes,dt)
            
        # Moves the robot arm to ready position for hitting on right
        if(self.time - self.hit_time + SWING_TIME >= 1):
            move_after_hit = True
            self.state = 'start'
            self.hitLeft = False
        
        
    # Function that checks whether or not the robot arm should hit a ball
    def checkReadyToHit(self):

        block1CurrPos = self.block1Fit(self.time)
        block2CurrPos = self.block2Fit(self.time)
        block3CurrPos = self.block3Fit(self.time)
        
        # Estimate for the position of blocks in future
        if (not self.hitLeft):
            block1NextPos = self.block1Fit(self.time + RIGHT_INTERVALS[0])
            block2NextPos = self.block2Fit(self.time + RIGHT_INTERVALS[1])
            block3NextPos = self.block3Fit(self.time + RIGHT_INTERVALS[2])
        else:
            block1NextPos = self.block1Fit(self.time + LEFT_INTERVALS[0])
            block2NextPos = self.block2Fit(self.time + LEFT_INTERVALS[1])
            block3NextPos = self.block3Fit(self.time + LEFT_INTERVALS[2])
      
        # Check if any of the blocks are going to hit the ball in next 2 seconds
        if(self.ballPresent):
            posBlock1 = block1NextPos + self.width[0]*0.5 
            posBlock2 = block2NextPos + self.width[1]*0.5 
            posBlock3 = block3NextPos + self.width[2]*0.5 

           # Hit right when all blocks are less than the target
            if(posBlock1 < RIGHT_TARGET): 
                if(posBlock2 < RIGHT_TARGET):
                   if(posBlock3 < RIGHT_TARGET): 
                        self.state = 'hitBall'
                        self.hit_time = self.time
                        pass
                    
             # hit Ball on left
            if(block1NextPos + self.width[0]*0.5 > LEFT_TARGET ): 
                if(block2NextPos + self.width[1]*0.5 > LEFT_TARGET):
                   if(block3NextPos + self.width[2]*0.5 > LEFT_TARGET): 
                        self.hitLeft = True
                        self.qdes = [0.0, 1.47, -1.08, 1.187, 1.5708, 0.0, 0.0]
                        self.robotController.addMilestone(self.qdes)
                       # only hit when robot arm reaches pre left hit position
                        if(self.robotController.getCommandedConfig()==self.qdes):
                            self.state = 'hitBall'
                            self.hit_time = self.time
                            pass
                        
        # update time when the ball was hit       
        if(move_after_hit):
            if(self.time - self.hit_time >=3):
                self.state = 'waiting'
  
        
    # Function that returns a sine fit curve from the position data 
    #obtained of the blocks 
    def sinFit(self,data):
        '''Fit sin to the input time sequence, and return fitting parameters "amp", "omega", "phase", "offset", "freq", "period" and "fitfunc"'''
        tt = np.linspace(0, WAITTIME, len(data))
        yy = np.array(data)
        ff = np.fft.fftfreq(len(tt), (tt[1]-tt[0]))   # assume uniform spacing
        Fyy = abs(np.fft.fft(yy))
        guess_freq = abs(ff[np.argmax(Fyy[1:])+1])   # excluding the zero frequency "peak", which is related to offset
        guess_amp = np.std(yy) * 2.**0.5
        guess_offset = np.mean(yy)
        guess = np.array([guess_amp, 2.*np.pi*guess_freq, 0., guess_offset])
    
        def sinfunc(t, A, w, p, c):  
            return A * np.sin(w*t + p) + c
        popt, pcov = sp.optimize.curve_fit(sinfunc, tt, yy, p0=guess)
        A, w, p, c = popt
        f = w/(2.*np.pi)
        fitfunc = lambda t: A * np.sin(w*t + p) + c
        return fitfunc
        
    def initVis(self):
        """If you want to do some visualization, initialize it here.
        TODO: You may consider visually debugging some of your code here, along with updateVis().
        """
        pass
    
    def updateVis(self):
        """This gets called every control loop.
        TODO: You may consider visually debugging some of your code here, along with initVis().

        For example, to draw a ghost robot at a given configuration q, you can call:
          kviz.add_ghost()  (in initVis)
          kviz.set_ghost(q) (in updateVis)

        The current code draws gravity-inflenced arcs leading from all the
        object position / velocity estimates from your state estimator.  Event C
        folks should set gravity=0 in the following code.
        """
        if self.objectEstimates:
            for o in self.objectEstimates.objects:
                #print(o.name)
                kviz.update_sphere("objballect_est"+str(o.name),o.x[0],o.x[1],o.x[2],0.03)
                kviz.set_color("object_est"+str(o.name),o.name[0],o.name[1],o.name[2])
                #draw an arc
                trace = []
                x = [o.x[0],o.x[1],o.x[2]]
                v = [o.x[3],o.x[4],o.x[5]]
                if event=='C': gravity = 0
                else: gravity = 9.8
                for i in range(20):
                    t = i*0.05
                    trace.append(vectorops.sub(vectorops.madd(x,v,t),[0,0,0.5*gravity*t*t]))
                
                
                kviz.update_polyline("object_trace"+str(o.name),trace);
                kviz.set_color("object_trace"+str(o.name),o.name[0],o.name[1],o.name[2])