# based on adafruit's arduino implementation, and
# https://github.com/pololu/LSM303/blob/master/LSM303/LSM303.cpp
import struct
import math
import time
from Adafruit_I2C import Adafruit_I2C
                                                   # DEFAULT    TYPE
LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20   # 00000111   rw
LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21   # 00000000   rw
LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22   # 00000000   rw
LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23   # 00000000   rw
LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24   # 00000000   rw
LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25   # 00000000   rw
LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26   # 00000000   r
LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27   # 00000000   r
LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28
LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29
LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A
LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B
LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C
LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D
LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E
LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F
LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30
LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31
LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32
LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33
LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34
LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35
LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36
LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37
LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38
LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39
LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A
LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B
LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C
LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
                                                
LSM303_REGISTER_MAG_CRA_REG_M             = 0x00
LSM303_REGISTER_MAG_CRB_REG_M             = 0x01
LSM303_REGISTER_MAG_MR_REG_M              = 0x02
LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03
LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04
LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05
LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06
LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07
LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08
LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09
LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A
LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B
LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C
LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31
LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32

      
LSM303_MAGGAIN_1_3                        = 0x20  # +/- 1.3
LSM303_MAGGAIN_1_9                        = 0x40  # +/- 1.9
LSM303_MAGGAIN_2_5                        = 0x60  # +/- 2.5
LSM303_MAGGAIN_4_0                        = 0x80  # +/- 4.0
LSM303_MAGGAIN_4_7                        = 0xA0  # +/- 4.7
LSM303_MAGGAIN_5_6                        = 0xC0  # +/- 5.6
LSM303_MAGGAIN_8_1                        = 0xE0  # +/- 8.1

LSM303_ADDRESS_ACCEL  =        (0x32 >> 1)         # 0011001x
LSM303_ADDRESS_MAG    =        (0x3C >> 1)         # 0011110x
                                                     
SENSORS_GRAVITY_EARTH        = 9.80665                # Earth's gravity in m/s^2 
SENSORS_GRAVITY_MOON         = 1.6                    # The moon's gravity in m/s^2 
SENSORS_GRAVITY_SUN          = 275.0                  # The sun's gravity in m/s^2 
SENSORS_GRAVITY_STANDARD     = SENSORS_GRAVITY_EARTH  #
SENSORS_MAGFIELD_EARTH_MAX   = 60.0                   # Maximum magnetic field on Earth's surface 
SENSORS_MAGFIELD_EARTH_MIN   = 30.0                   # Minimum magnetic field on Earth's surface 
SENSORS_PRESSURE_SEALEVELHPA = 1013.25                # Average sea level pressure is 1013.25 hPa 
SENSORS_DPS_TO_RADS          = 0.017453293            # Degrees/s to rad/s multiplier */
SENSORS_GAUSS_TO_MICROTESLA  = 100.0                  # Gauss to micro-Tesla multiplier 
    
class LSM303(object):
    
    def __init__(self):
        self.lsm303Accel_MG_LSB     = 0.001
        self.lsm303Mag_Gauss_LSB_XY = 1100.0;  # Varies with gain
        self.lsm303Mag_Gauss_LSB_Z  = 980.0;   # Varies with gain  

        self.i2cMag = Adafruit_I2C(LSM303_ADDRESS_MAG)
        self.i2cMag.write8(LSM303_REGISTER_MAG_MR_REG_M, 0x00)
        
        self.i2cAccel = Adafruit_I2C(LSM303_ADDRESS_ACCEL)
        self.i2cAccel.write8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27)
        self.i2cAccel.write8(LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x00)
        self.i2cAccel.write8(LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x08) # DLHC: enable high resolution mode
        
        self.magDataX = None
        self.magDataY = None
        self.magDataZ = None

        self.acclDataX = None
        self.acclDataY = None
        self.acclDataZ = None       

        self.magGain = None
        
        self.magMinX = -580
        self.magMinY = -620
        self.magMinZ = -76
        self.magMaxX = 533
        self.magMaxY = 113
        self.magMaxZ = 939


        self.magMinY = -380
        
        self.setMagGain(LSM303_MAGGAIN_1_3)

    def setMagGain(self, gain):
        self.i2cMag.write8(LSM303_REGISTER_MAG_CRB_REG_M, gain)
        self.magGain = gain
        
        gainsSwitch = {
            LSM303_MAGGAIN_1_3:(1100, 980),
            LSM303_MAGGAIN_1_9:(855, 760),
            LSM303_MAGGAIN_2_5:(670, 600),
            LSM303_MAGGAIN_4_0:(450, 400),
            LSM303_MAGGAIN_4_7:(400, 255),
            LSM303_MAGGAIN_5_6:(330, 295),
            LSM303_MAGGAIN_8_1:(230, 205)
        }
        self.lsm303Mag_Gauss_LSB_XY, self.lsm303Mag_Gauss_LSB_Z = gainsSwitch[gain]
        
    
    def getMagEvent(self):
        self.readMag()
        x = self.magDataX * 1.0 / self.lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
        y = self.magDataY * 1.0 / self.lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA
        z = self.magDataZ * 1.0 / self.lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA
        return (x,y,z)
    
    def getHeading(self):
        return self.getHeadingFromEvent(self.getMagEvent())
        
    def getHeadingFromEvent(self, event):
        x,y,_ = event
        angle = (math.atan2(y,x) * 180) / math.pi;
        
        if angle < 0:
            return 360+angle
        return angle
        
    def readMag(self):
        # Read the magnetometer
        bytesList = self.i2cMag.readList(LSM303_REGISTER_MAG_OUT_X_H_M, 6)
        byteData = "".join([chr(x) for x in bytesList])
        
        # Shift values to create properly formed integer (low byte first
        x,z,y = struct.unpack(">hhh", byteData)
        
        self.magDataX = x
        self.magDataY = y
        self.magDataZ = z

        
    def readAccel(self):
        # Read the accelometer
        bytesList = self.i2cAccel.readList(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, 6)
        byteData = "".join([chr(x) for x in bytesList])
        
        # Shift values to create properly formed integer (low byte first
        x,y,z = struct.unpack("<hhh", byteData)

        self.acclDataX = x >> 4
        self.acclDataY = y >> 4
        self.acclDataZ = z >> 4

    
    def getAcclEvent(self):
        self.readAccel()
        x = self.acclDataX * self.lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD
        y = self.acclDataY * self.lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD
        z = self.acclDataZ * self.lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD
        return (x,y,z)

def d(*args):
    return sum([x*x for x in args])**.5

if __name__ == '__main__':
    l = LSM303()
    minx, miny,minz = 0,0,0
    maxx, maxy,maxz = 0,0,0
    while True:
        e = l.getMagEvent()
        x,y,z = e
        mx,my,mz = e
        angle = l.getHeadingFromEvent(e)
        xyz = "(" + ", ".join(["{: 7.2f}"]*3) + ")"
        print "Heading = " , "{: 7.2f}".format(angle) , xyz.format(*e), "; d(x, y) = {: 7.2f}".format(d(x,y)) , "; d(x, y, z) = {: 7.2f}".format(d(x,y,z)), "; rawd(x,y,z)=",d(l.magDataX, l.magDataY, l.magDataZ)
            
        minx, miny,minz = min(minx,l.magDataX), min(miny,l.magDataY),min(minz,l.magDataZ)
        maxx, maxy,maxz = max(maxx,l.magDataX),max(maxy,l.magDataY),max(maxz,l.magDataZ)
        print "raws", l.magDataX, l.magDataY,l.magDataZ
        print "mins", minx, miny,minz
        print "maxs", maxx, maxy,maxz
        
        e = l.getAcclEvent()
        x,y,z=e
        # d should be earth gravity, but who knows...
        # http://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
        print "AccelData: ", xyz.format(*e),
        da = d(x, y, z)
        if da > 0:
            x1,y1,z1=x/da,y/da,z/da        
            print "d",da,"tilt x", "%.3f" % (math.asin(x1)*180/math.pi), "roll y", "%.3f"%(math.asin(y1)*180/math.pi)
        
        rollRadians  = math.asin(y1)
        pitchRadians = math.asin(x1)
  
        cosRoll  = math.cos(rollRadians);
        sinRoll  = math.sin(rollRadians);  
        cosPitch = math.cos(pitchRadians);
        sinPitch = math.sin(pitchRadians);
        
        # The tilt compensation algorithem.
        Xh = mx * cosPitch + mz * sinPitch
        Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch
        print "XXX" ,Xh ,mx , cosPitch , mz , sinPitch
        heading = math.atan2(Yh, Xh)
        h = heading*180/math.pi
        h = 360 + h if h < 0 else h
        print "Compensated Heading = " , "{: 7.2f}".format(h)
        
        # N E S W
        # mine:  [345,175,240,290]
        # phone: [355,130,220,300]
        # need to streach mine to 90 deg each:
        #        [345, 345+90,...]
        a = 175
        b = 240
        seg = 1
        newAngle = None
        for a,b in [(175,240), (240,290),(290,345)]:
            if a <= angle <= b:
                where = (angle-a) / (a-b)
                newAngle = 345+90*seg+90*where
                newAngle = newAngle % 360
            seg += 1
            
        a = 345
        b = 175
        segLen = (360-a+b)
        if newAngle is None:
            # then it is in the last range...
            if angle < 360:
                where = (angle-a)/segLen
            else: # < 175
                where = (angle + (360-a))/segLen

            newAngle = 345+90*where
            newAngle = newAngle % 360

        #print "Kombina Heading = " , "{: 7.2f}".format(newAngle)
        
        # calibrated heading:
        
        x = (l.magDataX - l.magMinX)*1.0 / (l.magMaxX - l.magMinX)
        y = (l.magDataY - l.magMinY)*1.0 / (l.magMaxY - l.magMinY)
        z = (l.magDataZ - l.magMinZ)*1.0 / (l.magMaxZ - l.magMinZ)
        x = x*2 -1
        y = y*2 -1
        z = z*2 -1
        heading = math.atan2(y,x)
        h = heading*180/math.pi
        h = 360 + h if h < 0 else h
        #print "Calibreated Heading = " , "{: 7.2f}".format(h)
        print ""        
        time.sleep(1)

