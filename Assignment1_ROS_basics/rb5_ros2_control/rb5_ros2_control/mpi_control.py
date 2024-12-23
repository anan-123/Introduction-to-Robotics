"""
This file include code that control the robot motors
"""


from megapi import MegaPi
import math
# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left
waypoints= [[0,0,0],[-1,0,0],[-1,1,1.57],[-2,1,0],[-2,2,-1.57],[-1,1,-0.78],[0,0,0]]

class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   
        self.theta = 0
        self.theta_c = 0
    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,-vbl)
        self.bot.motorRun(self.mbr,vbr)


    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)
    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(speed, speed, speed, speed)

    def carBack(self, speed):
        if self.verbose:
            print("CAR BACK:")
        self.setFourMotors(-speed, -speed, -speed, -speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carSlideLeft(self, speed):
        if self.verbose:
            print("CAR SLIDELEFT:")
        self.setFourMotors(-speed, speed, speed, -speed)

    def carSlideRight(self, speed):
        if self.verbose:
            print("CAR SLIDERIGHT:")
        self.setFourMotors(speed, -speed, -speed, speed)

    def carLeftFront(self, speed):
        if self.verbose:
            print("CAR LEFTFRONT:")
        self.setFourMotors(0, speed, speed, 0)
    
    def carLeftBack(self, speed):
        if self.verbose:
            print("CAR LEFTBACK:")
        self.setFourMotors(-speed, 0, 0, -speed)

    def carRightFront(self, speed):
        if self.verbose:
            print("CAR RIGHTFRONT:")
        self.setFourMotors(speed, 0, 0, speed)
    
    def carRightBack(self, speed):
        if self.verbose:
            print("CAR RIGHTBACK:")
        self.setFourMotors(0, -speed, -speed, 0)
    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()
    def calculate_v(self,x1,y1,x2,y2,t):
        Kv= 20
        Kh = 10
        self.distance = math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))
        v = Kv * (math.sqrt(pow(x1-x2,2)+pow(y1-y2,2)))
        self.theta_c = math.atan((y2-y1)/(x2-x1))
        self.theta = self.theta + self.theta_c
        vt = Kh * self.theta
        print("V = ",v," theta_c = ",self.theta_c," theta = ",self.theta," vt = ",vt)
        self.move2point(v,vt)
    def move2point(self, v, vt):
        tv = 60
        th = 60
        t1 = (self.distance/v) *tv 
        t2 = (self.theta_c/vt) * th
        print("t1 = ",t1," t2 = ",t2)
        mpi_ctrl.carRotate(vt)
        time.sleep(t2)
        mpi_ctrl.carStraight(v)
        time.sleep(t1)

if __name__ == "__main__":
    import time
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    # time.sleep(1)
    # mpi_ctrl.carStraight(30)
    # time.sleep(4)
    # mpi_ctrl.carSlide(30)
    # time.sleep(1)
    # mpi_ctrl.carRotate(30)
    for i in range(2, len(waypoints)):
        mpi_ctrl.calculate_v(waypoints[i-1][0], waypoints[i-1][1], waypoints[i][0], waypoints[i][1])
        break

    time.sleep(1)
    mpi_ctrl.carStop()
    mpi_ctrl.close()

    self.theta = 0  # keep track of current orrientation of the robot
    # for every pair of waypoints
    theta_c = math.atan2(y2-y1, x2-x1)  # line angle
    k1 = 1  # for rotate
    self.carRotate(k1 * (theta_c - self.theta))   # direction of the rotation may be wrong
    self.theta = theta_c
    distance = ((x1-x2)** 2+(y1-y2)**2) ** 0.5
    k2 = 1 # for straight
    self.carSTRAIGHT(k2 * distance)
    if self.theta != t:
        self.carRotate(k1 *(t - self.theta))
        self.theta = t

def calculate_v(self,x1,y1,x2,y2,t):
        print("hi1")
        Kv= 35
        Kh = 27
        tv = 150
        th = 45

        distance = (((x1-x2)**2+(y1-y2)**2))**0.5
        print(distance)
        v = int(Kv * (((x1-x2)**2+(y1-y2)**2))**0.5 )
        t1 =int( (distance/v) *tv)
        #rotate
        theta_c = math.atan2(y2-y1, x2-x1)
        vt =int( Kh*(theta_c - self.theta))
        self.carRotate(-vt)
        time.sleep(2)
        #update_orientation
        self.theta = theta_c
        #straight
        self.carStraight(-v)
        time.sleep(t1+1)
        print("V = ",v," theta_c = ",t," vt = ",vt)
        #rotate to final orientation
        print("t1 = ", t1)
        if t!= self.theta:
           # t2 =int( (t/vt * th) #change back to theta_c
            self.carRotate(-Kh *(t - self.theta))
            time.sleep(2)
     