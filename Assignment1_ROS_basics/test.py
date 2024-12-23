
from megapi import MegaPi
import math
# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

waypoints= [[0,0,0],[-1,0,0],[-1,1,1.57],[-2,1,0],[-2,2,1.57],[-1,1,-0.78],[0,0,0]]
#waypoints = [[0,0,0],[-1,0,0],[-1,1,1.57],[-2,1,0],[-2,2,-1.57],[-1,1,-0.78],[0,0,0]]
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

    def check_theta(self):
        if self.theta_c >= 3.14:
            self.theta_c -= 3.14
        elif self.theta_c < 0:
            self.theta_c += 3.14

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
       # self.setFourMotors(-speed, speed, -speed, speed)
        self.setFourMotors(speed, speed, speed, speed)

    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
      #  self.setFourMotors(speed, speed, speed, speed)
        self.setFourMotors(-speed, speed, -speed, speed)
    def carLeftFront(self, speed):
        if self.verbose:
            print("CAR LEFTFRONT:")
     #   self.setFourMotors(speed, speed, -speed, -speed)
        self.setFourMotors(0, speed, speed,0)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
     #   self.setFourMotors(speed, speed, -speed, -speed)
        self.setFourMotors(-speed, speed, speed, -speed)


    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
            if v_rotate<=0.1:
                v_straight= v_straight*50
                v_slide = v_slide * 50
            else:
                v_rotate = v_rotate * 50 / 1.57
                v_slide = v_slide *  50
                v_straight = v_straight*50
            print("vstraight =",v_straight)
            print("vrotate =",v_rotate)
            print("vslide=",v_slide)
            if v_rotate >0.1:
                 speed = int(v_rotate)
                 self.setFourMotors(-speed, speed, -speed, speed)
                 time.sleep(2)
            if v_straight >0.1:
                 speed = int(v_straight)
                 self.setFourMotors(speed, speed, speed, speed)
                 time.sleep(3)
            if v_slide >0.1:
                 speed = int(v_slide)
                 self.setFourMotors(-speed, speed, speed, -speed)
                 time.sleep(2)

            v_rotate=0
            v1 = v_rotate+v_straight+v_slide
            v2 = -v_rotate+v_straight-v_slide
            v3 = v_rotate+v_straight+v_slide
            v4 = -v_rotate+v_straight-v_slide

            print(v1)
            print(v2)
            print(v3)
            print(v4)
           # self.setFourMotors(int(v1), int(v2), int(v3), int(v4))
            #time.sleep(3)



    def close(self):
        self.bot.close()
        self.bot.exit()

    def calculate_v(self,x1,y1,x2,y2,t):
        print("hi1")
        Kv= 35
        Kh = 27
        #self.distance=10
        self.distance = (((x1-x2)**2+(y1-y2)**2))**0.5
        print(self.distance)
        v = int(Kv * (((x1-x2)**2+(y1-y2)**2))**0.5 )
  #      angle = math.atan2((y2-y1),(x2-x1))
   #     print("angle = ",angle)
        vt =int( Kh*t)
        print("V = ",v," theta_c = ",t," vt = ",vt)
        tv = 150
        th = 45
        t1 =int( (self.distance/v) *tv)
        print("t1 = ", t1)
        if t!= 0:
           # t2 =int( (t/vt * th) #change back to theta_c
            self.carRotate(-vt)
            time.sleep(2)
           # print("t1 = ",t1," t2 = ",t2)
        #self.carRotate(vt)
        #time.sleep(t2)
        self.carStraight(-v)
        time.sleep(t1+1)
        #return vt, v, t1, t2


if __name__ == "__main__":
    import time
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)
    time.sleep(1)
   # mpi_ctrl.carStraight(60)
   # time.sleep(4)
   # mpi_ctrl.carSlide(40)
  #  time.sleep(1)
   # mpi_ctrl.carRotate(30)
   # time.sleep(4)
    #mpi_ctrl.carLeftFront(40)
   # time.sleep(1)
 #   mpi_ctrl.setFourMotors(21,21,19,23)
   # time.sleep(4)
 #   mpi_ctrl.setFourMotors(24,24,2,24)
  #  time.sleep(4)
  #  mpi_ctrl.setFourMotors(26,26,17,26)
#    time.sleep(4)
 #   mpi_ctrl.setFourMotors(28,28,18,28)
     #   time.sleep(4)

    #mpi_ctrl.carMixed(30, 30, 30)
    for i in range(1, len(waypoints)):
        mpi_ctrl.calculate_v(waypoints[i-1][0], waypoints[i-1][1], waypoints[i][0], waypoints[i][1],waypoints[i][2]-waypoints[i-1][2])

       # mpi_ctrl.carMixed(waypoints[i][0]-waypoints[i-1][0],waypoints[i][2]-waypoints[i-1][2],waypoints[i][1]-waypoints[i-1][1])

     #mpi_ctrl.carStraight(int(v))
        #time.sleep(t1)
        #break

   # time.sleep(4)
    mpi_ctrl.carStop()
    mpi_ctrl.close()
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
~                                                                                                                                                                                                                  
                                                                                                                                                                                                 175,1         Bot
