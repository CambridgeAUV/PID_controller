
import time

class PIDControl():

    def __init__(self, kp, ki, kd, isAngle):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.isAngle = isAngle
        self.first = True
        self.integralError = 0
        self.prevError = 0


    def getErrorAngle(self, target, current):
        diff = (target - current)%360
        if diff > 180:
            diff = diff - 360
        return diff
    

    def getError(self, target, current):
        return target - current


    def clamp(self, val, minVal, maxVal):
        if val > maxVal:
            return maxVal
        elif val < minVal:
            return minVal
        return val

    def getDemand(self, target, current):

        if self.first:
            self.prevTime = time.time() 
            return

        # time since last call in seconds
        dt = time.time() - self.prevTime
        self.prevTime = time.time()

        # prevent division by zero
        if dt == 0:
            return

        if(self.isAngle):
            proportionError = self.getErrorAngle(target, current)
        else:
            proportionError = self.getError(target, current)
        proportionError = self.clamp(proportionError, -errorMax, errorMAX)

        self.integralError += error * dt
        self.integralError = clamp(integralError, -errorMax, errorMAX)

        derivativeError = (error - self.prevError) / dt
        derivativeError = clamp(derivativeError, -errorMax, errorMAX)

        demand = (self.kp * proportionError + 
                 self.ki * self.integralError + 
                 self.kd * derivativeError)

        return demand
