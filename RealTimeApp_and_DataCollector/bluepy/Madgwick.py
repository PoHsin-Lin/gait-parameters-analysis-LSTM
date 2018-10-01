import math

class MadgwickClass:

    def __init__(self):
        self.Q = [None]*4
        self.RAD2DEG = 57.2957795

        self.Q[0] = 0.7071
        self.Q[1] = -0.7071
        self.Q[2] = 0
        self.Q[3] = 0

        self.Q0Q0 = 1.0
        self.Q0Q1 = 0.0
        self.Q0Q2 = 0.0
        self.Q0Q3 = 0.0
        self.Q1Q1 = 0.0
        self.Q1Q2 = 0.0
        self.Q1Q3 = 0.0
        self.Q2Q2 = 0.0
        self.Q2Q3 = 0.0
        self.Q3Q3 = 0.0

        self.beta = 0.1

    def MadgwickAHRSupdateIMU(self,gx,gy,gz,ax,ay,az,SamplePeriod):
        
        
        # Rate of change of Quaternion from gyroscope
        QDot1 = 0.5 * (-self.Q[1] * gx - self.Q[2] * gy - self.Q[3] * gz)
        QDot2 = 0.5 * (self.Q[0] * gx + self.Q[2] * gz - self.Q[3] * gy)
        QDot3 = 0.5 * (self.Q[0] * gy - self.Q[1] * gz + self.Q[3] * gx)
        QDot4 = 0.5 * (self.Q[0] * gz + self.Q[1] * gy - self.Q[2] * gx)

        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):

            # Normalise accelerometer measurement
            recipNorm = 1/math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm   
            
            # Auxiliary variables to avoid repeated arithmetic
            _2Q0 = 2.0 * self.Q[0]
            _2Q1 = 2.0 * self.Q[1]
            _2Q2 = 2.0 * self.Q[2]
            _2Q3 = 2.0 * self.Q[3]
            _4Q0 = 4.0 * self.Q[0]
            _4Q1 = 4.0 * self.Q[1]
            _4Q2 = 4.0 * self.Q[2]
            _8Q1 = 8.0 * self.Q[1]
            _8Q2 = 8.0 * self.Q[2]
            self.Q0Q0 = self.Q[0] * self.Q[0]
            self.Q1Q1 = self.Q[1] * self.Q[1]
            self.Q2Q2 = self.Q[2] * self.Q[2]
            self.Q3Q3 = self.Q[3] * self.Q[3]

        # Gradient decent algorithm corrective step
            s0 = _4Q0 * self.Q2Q2 + _2Q2 * ax + _4Q0 * self.Q1Q1 - _2Q1 * ay
            s1 = _4Q1 * self.Q3Q3 - _2Q3 * ax + 4.0 * self.Q0Q0 * self.Q[1] - _2Q0 * ay - _4Q1 + _8Q1 * self.Q1Q1 + _8Q1 * self.Q2Q2 + _4Q1 * az
            s2 = 4.0 * self.Q0Q0 * self.Q[2] + _2Q0 * ax + _4Q2 * self.Q3Q3 - _2Q3 * ay - _4Q2 + _8Q2 * self.Q1Q1 + _8Q2 * self.Q2Q2 + _4Q2 * az
            s3 = 4.0 * self.Q1Q1 * self.Q[3] - _2Q1 * ax + 4.0 * self.Q2Q2 * self.Q[3] - _2Q2 * ay
           
            # normalise step magnitude
            recipNorm = 1/math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) 
            s0 *= recipNorm
            s1 *= recipNorm
            s2 *= recipNorm
            s3 *= recipNorm

            # Apply feedback step
            QDot1 -= self.beta * s0
            QDot2 -= self.beta * s1
            QDot3 -= self.beta * s2
            QDot4 -= self.beta * s3
    

        # Integrate rate of change of Quaternion to yield Quaternion
        self.Q[0] += QDot1 * (1.0 * SamplePeriod)
        self.Q[1] += QDot2 * (1.0 * SamplePeriod)
        self.Q[2] += QDot3 * (1.0 * SamplePeriod)
        self.Q[3] += QDot4 * (1.0 * SamplePeriod)

        # Normalise Quaternion
        recipNorm = 1/math.sqrt(self.Q[0] * self.Q[0] + self.Q[1] * self.Q[1] + self.Q[2] * self.Q[2] + self.Q[3] * self.Q[3])
        
        self.Q[0] *= recipNorm
        self.Q[1] *= recipNorm
        self.Q[2] *= recipNorm
        self.Q[3] *= recipNorm



        self.Q0Q0 = self.Q[0] * self.Q[0]
        self.Q0Q1 = self.Q[0] * self.Q[1]
        self.Q0Q2 = self.Q[0] * self.Q[2]
        self.Q0Q3 = self.Q[0] * self.Q[3]
        self.Q1Q1 = self.Q[1] * self.Q[1]
        self.Q1Q2 = self.Q[1] * self.Q[2]
        self.Q1Q3 = self.Q[1] * self.Q[3]
        self.Q2Q2 = self.Q[2] * self.Q[2]
        self.Q2Q3 = self.Q[2] * self.Q[3]
        self.Q3Q3 = self.Q[3] * self.Q[3]
        
        #print self.Q
        #print self.quatern2euler()
    def quaternLCStoGCS(self,vector):
        
        tmpProduct = product(self.Q,vector)
        conjugateQ = self.Q
        
        for i in range(1,4):
            conjugateQ[i] = conjugateQ[i]*-1

        productAns = product(tmpProduct,conjugateQ)

        return [productAns[1],productAns[2],productAns[3]]

    def quatern2ReverseRotMat(self):
        rotmat = [[0,0,0],[0,0,0],[0,0,0]]

        rotmat[0][0] = 1 - 2*( self.Q2Q2 + self.Q3Q3 ) 
        rotmat[0][1] = 2*( self.Q1Q2 - self.Q0Q3 )
        rotmat[0][2] = 2*( self.Q1Q3 + self.Q0Q2 )
        rotmat[1][0] = 2*( self.Q1Q2 + self.Q0Q3)
        rotmat[1][1] = 1 - 2*( self.Q1Q1 + self.Q3Q3 ) 
        rotmat[1][2] = 2*( self.Q2Q3 - self.Q0Q1 )
        rotmat[2][0] = 2*( self.Q1Q3 - self.Q0Q2 )
        rotmat[2][1] = 2*( self.Q2Q3 + self.Q0Q1)
        rotmat[2][2] = 1 - 2*( self.Q1Q1 + self.Q2Q2 ) 
      #  print rotmat[2]
        return rotmat

    def quatern2euler(self):
        angles = [None] * 3
                    
        angles[0] = math.atan2(2 * self.Q1Q2 + 2 * self.Q0Q3, -2 * self.Q2Q2 - 2 * self.Q3Q3 + 1) # yaw
        angles[1] = -1*math.asin(2 * self.Q1Q3 - 2 * self.Q0Q2) # pitch
        angles[2] = math.atan2(2 * self.Q2Q3 + 2 * self.Q0Q1, -2 * self.Q1Q1 - 2 * self.Q2Q2 + 1) #roll

        angles[0] *= self.RAD2DEG
        angles[1] *= self.RAD2DEG
        angles[2] *= self.RAD2DEG

        angles[0] = angles[0]*-1
        angles[1] = angles[1]*-1
        angles[2] = angles[2]*-1
        
        '''
        self.count = self.count + 1
        print("NUM:",self.count)
        '''
        #print("ANGLE",angles)
        return angles

    def eular2quatern(self,angles):

        t0 = math.cos(angles[0]*0.5/self.RAD2DEG)
        t1 = math.sin(-1*angles[0]*0.5/self.RAD2DEG)        
        t2 = math.cos(angles[2]*0.5/self.RAD2DEG)#1
        t3 = math.sin(-1*angles[2]*0.5/self.RAD2DEG)#0     
        t4 = math.cos(angles[1]*0.5/self.RAD2DEG)#1
        t5 = math.sin(-1*angles[1]*0.5/self.RAD2DEG)#0
            

        #Qtmp = self.Q

        self.Q[0] = t0 * t2 * t4 + t1 * t3 * t5 # t0*1*1 + t1*0*0 = t1
        self.Q[1] = t0 * t3 * t4 - t1 * t2 * t5 # t0*0*1 - t1*t2*0 = 0
        self.Q[2] = t0 * t2 * t5 + t1 * t3 * t4 # t0*1*0 + t1*0*1 = 0
        self.Q[3] = (t1 * t2 * t4 - t0 * t3 * t5) # t1*1*1 - t0*0*0 = t1

        #reNormalise
        recipNorm = 1/math.sqrt(self.Q[0] * self.Q[0] + self.Q[1] * self.Q[1] + self.Q[2] * self.Q[2] + self.Q[3] * self.Q[3]);

        self.Q[0] *= recipNorm
        self.Q[1] *= recipNorm
        self.Q[2] *= recipNorm
        self.Q[3] *= recipNorm

        self.Q0Q0 = self.Q[0] * self.Q[0]
        self.Q0Q1 = self.Q[0] * self.Q[1]
        self.Q0Q2 = self.Q[0] * self.Q[2]
        self.Q0Q3 = self.Q[0] * self.Q[3]
        self.Q1Q1 = self.Q[1] * self.Q[1]
        self.Q1Q2 = self.Q[1] * self.Q[2]
        self.Q1Q3 = self.Q[1] * self.Q[3]
        self.Q2Q2 = self.Q[2] * self.Q[2]
        self.Q2Q3 = self.Q[2] * self.Q[3]
        self.Q3Q3 = self.Q[3] * self.Q[3]

    def lulu(self):

        self.Q0Q0 = self.Q[0] * self.Q[0]
        self.Q0Q1 = self.Q[0] * self.Q[1]
        self.Q0Q2 = self.Q[0] * self.Q[2]
        self.Q0Q3 = self.Q[0] * self.Q[3]
        self.Q1Q1 = self.Q[1] * self.Q[1]
        self.Q1Q2 = self.Q[1] * self.Q[2]
        self.Q1Q3 = self.Q[1] * self.Q[3]
        self.Q2Q2 = self.Q[2] * self.Q[2]
        self.Q2Q3 = self.Q[2] * self.Q[3]
        self.Q3Q3 = self.Q[3] * self.Q[3]


def product(A,B):
    ans = [0]*4

    ans[0] = A[0]*B[0] - A[1]*B[1] - A[2]*B[2] - A[3]*B[3]
    ans[1] = A[0]*B[1] + A[1]*B[0] + A[2]*B[3] - A[3]*B[2]
    ans[2] = A[0]*B[2] - A[1]*B[3] + A[2]*B[0] + A[3]*B[1]
    ans[3] = A[0]*B[3] + A[1]*B[2] - A[2]*B[1] + A[3]*B[0]
 
    return ans
        
            
if __name__ == '__main__':
    
 
    testMad = MadgwickClass()

    testMad.Q = [0.701,0.701,0,0]
    testMad.lulu()
    testMad.quatern2ReverseRotMat()


