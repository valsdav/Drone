#!/usr/bin/python

import threading
import math
import sys
import time
import random

class Simulation():
    def __init__(self):
        
        # Threading variables
        self.lock = threading.Lock()
        self.physicsThread = threading.Thread(target=self.physicsAction, args=())
        self.forcesThread = threading.Thread(target=self.forcesAction, args=())

        self.runningFlag = True

        self.PI = 3.14159265
        self.physStep = 0.001 # 1 millisec
        
        #static parameters
        self.m = 4
        self.weight = self.m*9.81
        self.L_arm = 0.62
        self.I_X = 0.06
        self.I_Y = 0.06
        self.dt = 0.1
        
        #350 is the default number of step
        self.Fstep = self.weight / (4*350)
        self.dF = 0

        #dynamic variables
        self.angle_X = 0
        self.angle_Y = 0
        self.omega_X = 0
        self.omega_Y = 0
        self.alpha_X = 0
        self.alpha_Y = 0
        self.pos_X = 0
        self.pos_Y = 0
        self.pos_Z = 0
        self.vel_X = 0
        self.vel_Y = 0
        self.vel_Z = 0
        self.acc_X = 0
        self.acc_Y = 0
        self.acc_Z = 0

        #F1-F3 x axis. F2-F4 y axis
        self.F1_base = 350
        self.F2_base = 350
        self.F3_base = 350
        self.F4_base = 350
        self.F1 = self.F1_base
        self.F2 = self.F2_base
        self.F3 = self.F3_base
        self.F4 = self.F4_base
        self.Ftot_X = 0
        self.Ftot_Y = 0
        self.Ftot_Z = 0
        self.dTcorrection = 0
        self.time = 0
        self.prevTime = 0
        self.seconds = 0
        self.secondsCounter = 0

        #output history
        self.data = []
        self.curr_data = {}
        self.angle_X_history = []
        self.angle_Y_history = []
        self.omega_X_history = []
        self.omega_Y_history = []
        self.alpha_X_history = []
        self.alpha_Y_history = []
        self.pos_X_hist = []
        self.pos_Y_hist = []
        self.pos_Z_hist = []
        self.vel_X_hist = []
        self.vel_Y_hist = []
        self.vel_Z_hist = []
        self.acc_X_hist = []
        self.acc_Y_hist = []
        self.acc_Z_hist = []
        self.Ftot_X_hist = []
        self.Ftot_Y_hist = []
        self.Ftot_Z_hist = []
        self.F1_hist = []
        self.F2_hist = []
        self.F3_hist = []
        self.F4_hist = []
        self.deltaW = 0
        self.filename = ""
    
    def physicsAction(self):
        print "Seconds elapsed in the simulation:"
        while self.time < self.seconds:
            # Printing a timer
            if self.time > self.secondsCounter:
                print self.secondsCounter
                self.secondsCounter += 1

            self.curr_data = {}
            self.curr_data["t"] = self.time;
            # Extract dt from gaussian random number around physStep
            self.dt = random.gauss(self.physStep, self.physStep*0.25) #
            time.sleep(self.dt)
            self.lock.acquire()
            self.saveEnginesForces()
            self.calculateTotalForces()
            self.calculateCentreOfMassDynamic()
            self.calculateRotationalDynamic()
            self.lock.release()
            self.time += self.dt
            self.data.append(self.curr_data)
        print self.time
        print "Physics thread end."
        self.saveData()

    def forcesAction(self):
        while self.time < self.seconds:
            if self.prevTime + self.dTcorrection < self.time:
                self.lock.acquire()
                self.changeForcesWeight()
                self.lock.release()
                self.prevTime = self.time
        print "Forces thread end."


    def saveData(self):
        print "Writing output..."
        out = open(self.filename,"w")
    
        for key in self.curr_data:
            out.write(key+" ")
        out.write("\n")
    
        for single_data in self.data:
            for key in single_data:
                out.write(str(single_data[key]) + " ")
            out.write("\n")
        out.close()
        print "python print-drone.py angleX posZ velZ velX FtotX data"

    def activateThread(self):
        self.physicsThread.start()
        self.forcesThread.start()

    def changeForcesNormal(self):
        self.F1 = self.F1_base + (int(round(self.angle_X))* self.dF + self.dF* int(round(self.omega_X)))
        self.F3 = self.F3_base - (int(round(self.angle_X))* self.dF + self.dF* int(round(self.omega_X)))
        self.F2 = self.F2_base + (int(round(self.angle_Y))* self.dF + self.dF* int(round(self.omega_Y)))
        self.F4 = self.F4_base - (int(round(self.angle_Y))* self.dF + self.dF* int(round(self.omega_Y)))


    def changeForcesWeight(self):
        self.changeForcesNormal()
        angleFactor = math.cos(self.angle_X * self.PI / 180.0) * math.cos(self.angle_Y * self.PI / 180.0)
        deltaForce = self.weight/angleFactor - ((self.F1+self.F2+self.F3+self.F4)*self.Fstep) 
        deltaW = int(round(deltaForce / (4 * self.Fstep)))
        self.F1 += deltaW
        self.F2 += deltaW
        self.F3 += deltaW
        self.F4 += deltaW


    def saveEnginesForces(self):
        #saving real forces values
        self.curr_data["F1"] = self.F1
        self.curr_data["F2"] = self.F2
        self.curr_data["F3"] = self.F3
        self.curr_data["F4"] = self.F4
        self.curr_data["deltaW"] = self.deltaW


    def calculateRotationalDynamic(self):
        self.m_X = (self.F3 - self.F1)* self.Fstep * self.L_arm
        self.m_Y = (self.F4 - self.F2)* self.Fstep * self.L_arm
        self.alpha_X = self.m_X / self.I_X
        self.alpha_Y = self.m_Y / self.I_Y
        self.curr_data["alphaX"] = self.alpha_X
        self.curr_data["alphaY"] = self.alpha_Y

        self.omega_X += self.alpha_X * self.dt
        self.omega_Y += self.alpha_Y * self.dt
        self.curr_data["omegaX"] = self.omega_X
        self.curr_data["omegaY"] = self.omega_Y


        self.angle_X += self.omega_X*self.dt + 0.5 * self.alpha_X * self.dt * self.dt
        self.angle_Y += self.omega_Y*self.dt + 0.5 * self.alpha_Y * self.dt * self.dt
        self.curr_data["angleX"] = self.angle_X
        self.curr_data["angleY"] = self.angle_Y   

    def calculateCentreOfMassDynamic(self):
        self.acc_X = self.Ftot_X / self.m
        self.acc_Y = self.Ftot_Y / self.m
        self.acc_Z = self.Ftot_Z / self.m
        self.vel_X += self.acc_X * self.dt
        self.vel_Y += self.acc_Y * self.dt
        self.vel_Z += self.acc_Z * self.dt
        self.pos_X += self.vel_X * self.dt + 0.5 * self.acc_X * self.dt * self.dt
        self.pos_Y += self.vel_Y * self.dt + 0.5 * self.acc_Y * self.dt * self.dt
        self.pos_Z += self.vel_Z * self.dt + 0.5 * self.acc_Z * self.dt * self.dt
        self.curr_data["posX"] = self.pos_X
        self.curr_data["posY"] = self.pos_Y
        self.curr_data["posZ"] = self.pos_Z
        self.curr_data["velX"] = self.vel_X
        self.curr_data["velY"] = self.vel_Y
        self.curr_data["velZ"] = self.vel_Z
        self.curr_data["accX"] = self.acc_X
        self.curr_data["accY"] = self.acc_Y
        self.curr_data["accZ"] = self.acc_Z


    def calculateTotalForces(self):
        self.Ftot_X = (self.F1 + self.F2 + self.F3 + self.F4) * self.Fstep * math.sin(self.angle_X * self.PI / 180.0)
        self.Ftot_Y = (self.F1 + self.F2 + self.F3 + self.F4) * self.Fstep * math.sin(self.angle_Y * self.PI / 180.0)
        self.Ftot_Z = (self.F1 + self.F2 + self.F3 + self.F4) * self.Fstep * math.cos(self.angle_X * self.PI / 180.0) * math.cos(self.angle_Y * self.PI / 180.0) - self.weight
        self.curr_data["FtotX"] = self.Ftot_X
        self.curr_data["FtotY"] = self.Ftot_Y
        self.curr_data["FtotZ"] = self.Ftot_Z
        self.curr_data["Ftot"] = (self.F1 + self.F2 + self.F3 + self.F4) * self.Fstep

def main():
    len(sys.argv)
    print sys.argv
    if len(sys.argv) != 10:
        print "Insert: angle_X angle_Y omega_X omega_Y dF dTcorrection dTsimulation seconds name"
        print """Where:
- angle_X:      starting angle around X axis
- angle_Y:      starting angle around Y axis
- omega_X:      starting angular speed around X axis
- omega_Y:      starting angular speed around Y axis
- dF:           correction step applied to engines power
- dTcorrection: time in seconds between two engines corrections
- dTsimulation: not used (but value required)
- seconds:      simulated run time in seconds
- name:         output file name\n"""
        return 1
    
    seconds = 0
    
    mySimulation = Simulation()
    mySimulation.angle_X = float(sys.argv[1])
    mySimulation.angle_Y = float(sys.argv[2])
    mySimulation.omega_X = float(sys.argv[3])
    mySimulation.omega_Y = float(sys.argv[4])
    mySimulation.dF = float(sys.argv[5])
    mySimulation.dTcorrection = float(sys.argv[6])
    mySimulation.dt = float(sys.argv[7])
    mySimulation.seconds = float(sys.argv[8])
    mySimulation.filename = str(sys.argv[9])
    
    print "Starting process..."
    mySimulation.activateThread()

if __name__ == '__main__':
    main()



