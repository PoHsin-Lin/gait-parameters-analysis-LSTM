import math

class MahonyClass:

	def __init__(self):
		self.Q = [None]*4
		self.RAD2DEG = 57.2957795
		
		self.Q[0] = 1.0
		self.Q[1] = 0.0
		self.Q[2] = 0.0
		self.Q[3] = 0.0
		self.q0q0=1.0
		self.q0q1=0.0
		self.q0q2=0.0
		self.q0q3=0.0
		self.q1q1=0.0
		self.q1q2=0.0
		self.q1q3=0.0
		self.q2q2 = 0.0
		self.q2q3 = 0.0
		self.q3q3 = 0.0

		self.twoKp = 2.0 * 1.75
		self.twoKi = 2.0 * 0.1
		self.integralFBx = 0.0
		self.integralFBy = 0.0
		self.integralFBz = 0.0
		self.count=0;

	def MahonyIMU6Dof(self, gx, gy, gz, ax, ay, az, SamplePeriod):		
		halfex = halfey = halfez = 0

		if ax != 0.0 and ay != 0.0 and az != 0.0:
			#Normalise accelerometer measurement
			recipNorm = 1/math.sqrt(ax * ax + ay * ay + az * az)
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm


			self.q0q0 = self.Q[0] * self.Q[0]
			self.q0q1 = self.Q[0] * self.Q[1]
			self.q0q2 = self.Q[0] * self.Q[2]
			self.q0q3 = self.Q[0] * self.Q[3]
			self.q1q1 = self.Q[1] * self.Q[1]
			self.q1q2 = self.Q[1] * self.Q[2]
			self.q1q3 = self.Q[1] * self.Q[3]
			self.q2q2 = self.Q[2] * self.Q[2]
			self.q2q3 = self.Q[2] * self.Q[3]
			self.q3q3 = self.Q[3] * self.Q[3]
                        
			
			#Estimated direction of gravity
			halfvx = self.q1q3 - self.q0q2
			halfvy = self.q0q1 + self.q2q3
			#halfvz = q0q0 - 0.5f + q3q3
			halfvz = 0.5 * (self.q0q0 - self.q1q1 - self.q2q2 + self.q3q3)
			
			#Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (ay * halfvz - az * halfvy)
			halfey += (az * halfvx - ax * halfvz)
			halfez += (ax * halfvy - ay * halfvx)
			
		if halfex != 0.0 and halfey != 0.0 and halfez != 0.0:
			#Compute and apply integral feedback if enabled
			if self.twoKi > 0.0 :
				self.integralFBx += self.twoKi * halfex * SamplePeriod # integral error scaled by Ki
				self.integralFBy += self.twoKi * halfey * SamplePeriod
				self.integralFBz += self.twoKi * halfez * SamplePeriod
				gx += self.integralFBx #apply integral feedback
				gy += self.integralFBy
				gz += self.integralFBz
			else:
				self.integralFBx = 0.0 # prevent integral windup
				self.integralFBy = 0.0
				self.integralFBz = 0.0

			#Apply proportional feedback
			gx += self.twoKp * halfex
			gy += self.twoKp * halfey
			gz += self.twoKp * halfez
			
		#Integrate rate of change of quaternion
		gx *= (0.5 * SamplePeriod)   #pre-multiply common factors
		gy *= (0.5 * SamplePeriod)
		gz *= (0.5 * SamplePeriod)

		qa = self.Q[0];
		qb = self.Q[1];
		qc = self.Q[2];
		self.Q[0] += (-qb * gx - qc * gy - self.Q[3] * gz);
		self.Q[1] += (qa * gx + qc * gz - self.Q[3] * gy);
		self.Q[2] += (qa * gy - qb * gz + self.Q[3] * gx);
		self.Q[3] += (qa * gz + qb * gy - qc * gx);
        
		#Normalise quaternion
		recipNorm = 1/math.sqrt(self.Q[0] * self.Q[0] + self.Q[1] * self.Q[1] + self.Q[2] * self.Q[2] + self.Q[3] * self.Q[3]);
		self.Q[0] *= recipNorm;
		self.Q[1] *= recipNorm;
		self.Q[2] *= recipNorm;
		self.Q[3] *= recipNorm;
		

		self.q0q0 = self.Q[0] * self.Q[0]
		self.q0q1 = self.Q[0] * self.Q[1]
		self.q0q2 = self.Q[0] * self.Q[2]
		self.q0q3 = self.Q[0] * self.Q[3]
		self.q1q1 = self.Q[1] * self.Q[1]
		self.q1q2 = self.Q[1] * self.Q[2]
		self.q1q3 = self.Q[1] * self.Q[3]
		self.q2q2 = self.Q[2] * self.Q[2]
		self.q2q3 = self.Q[2] * self.Q[3]
		self.q3q3 = self.Q[3] * self.Q[3]
	def MahonyAHRSupdate(self, gx, gy, gz, ax, ay, az, mx, my, mz, SamplePeriod):
		#float recipNorm;
		#float hx, hy, bx, bz;
		#float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
		#float halfex, halfey, halfez;
		#float qa, qb, qc;
               
		# Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		#if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		#UpdateIMU(gx, gy, gz, ax, ay, az,SamplePeriod);
		#return;
		#}
		# Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)) :

			# Normalise accelerometer measurement
			recipNorm = 1/math.sqrt(ax * ax + ay * ay + az * az)
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm     
			# print ("Normalise ax:",ax)
			# print ("Normalise ay:",ay)
			# print ("Normalise az:",az)
			# Normalise magnetometer measurement
			recipNorm = 1/math.sqrt(mx * mx + my * my + mz * mz) 
			mx *= recipNorm
			my *= recipNorm
			mz *= recipNorm   
			# print ("Normalise mx:",mx)
			# print ("Normalise my:",my)
			# print ("Normalise mz:",mz)
                        #print("q0q0",self.q0q0)
			# Auxiliary variables to avoid repeated arithmetic
			self.q0q0 = self.Q[0] * self.Q[0]
			self.q0q1 = self.Q[0] * self.Q[1]
			self.q0q2 = self.Q[0] * self.Q[2]
			self.q0q3 = self.Q[0] * self.Q[3]
			self.q1q1 = self.Q[1] * self.Q[1]
			self.q1q2 = self.Q[1] * self.Q[2]
			self.q1q3 = self.Q[1] * self.Q[3]
			self.q2q2 = self.Q[2] * self.Q[2]
			self.q2q3 = self.Q[2] * self.Q[3]
			self.q3q3 = self.Q[3] * self.Q[3]
                        


			# Reference direction of Earth's magnetic field
			hx = 2.0 * (mx * (0.5 - self.q2q2 - self.q3q3) + my * (self.q1q2 - self.q0q3) + mz * (self.q1q3 + self.q0q2))
			hy = 2.0 * (mx * (self.q1q2 + self.q0q3) + my * (0.5 - self.q1q1 - self.q3q3) + mz * (self.q2q3 - self.q0q1))
			bx = math.sqrt(hx * hx + hy * hy)
			bz = 2.0 * (mx * (self.q1q3 - self.q0q2) + my * (self.q2q3 + self.q0q1) + mz * (0.5 - self.q1q1 - self.q2q2))
			

			# Estimated direction of gravity and magnetic field
			halfvx = self.q1q3 - self.q0q2
			halfvy = self.q0q1 + self.q2q3
			halfvz = self.q0q0 - 0.5 + self.q3q3
			halfwx = bx * (0.5 - self.q2q2 - self.q3q3) + bz * (self.q1q3 - self.q0q2)
			halfwy = bx * (self.q1q2 - self.q0q3) + bz * (self.q0q1 + self.q2q3)
			halfwz = bx * (self.q0q2 + self.q1q3) + bz * (0.5 - self.q1q1 - self.q2q2)



			# Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
			halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
			halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

			# Compute and apply integral feedback if enabled
			if(self.twoKi > 0.0):
				self.integralFBx = self.integralFBx + (self.twoKi * halfex *  SamplePeriod)	# integral error scaled by Ki
				self.integralFBy = self.integralFBx + (self.twoKi * halfey *  SamplePeriod)  #1.0 delete
				self.integralFBz = self.integralFBx + (self.twoKi * halfez * SamplePeriod)
				gx += self.integralFBx	# apply integral feedback
				gy += self.integralFBy
				gz += self.integralFBz

			
			else:
				self.integralFBx = 0.0	# prevent integral windup
				self.integralFBy = 0.0
				self.integralFBz = 0.0

			# Apply proportional feedback
			gx += self.twoKp * halfex
			gy += self.twoKp * halfey
			gz += self.twoKp * halfez
		

		
		# Integrate rate of change of quaternion
		gx *= (0.5 * SamplePeriod)		# pre-multiply common factors
		gy *= (0.5 * SamplePeriod)   #1.0 delete
		gz *= (0.5 * SamplePeriod)
		qa = self.Q[0]
		qb = self.Q[1]
		qc = self.Q[2]

		self.Q[0] += (-qb * gx - qc * gy - self.Q[3] * gz)
		self.Q[1] += (qa * gx + qc * gz - self.Q[3] * gy)
		self.Q[2] += (qa * gy - qb * gz + self.Q[3] * gx)
		self.Q[3] += (qa * gz + qb * gy - qc * gx)
		
		# Normalise quaternion
		recipNorm = 1/math.sqrt( self.Q[0] * self.Q[0] + self.Q[1] * self.Q[1] + self.Q[2] * self.Q[2] + self.Q[3] * self.Q[3])
		self.Q[0] *= recipNorm
		self.Q[1] *= recipNorm
		self.Q[2] *= recipNorm
		self.Q[3] *= recipNorm

		self.q0q0 = self.Q[0] * self.Q[0]
		self.q0q1 = self.Q[0] * self.Q[1]
		self.q0q2 = self.Q[0] * self.Q[2]
		self.q0q3 = self.Q[0] * self.Q[3]
		self.q1q1 = self.Q[1] * self.Q[1]
		self.q1q2 = self.Q[1] * self.Q[2]
		self.q1q3 = self.Q[1] * self.Q[3]
		self.q2q2 = self.Q[2] * self.Q[2]
		self.q2q3 = self.Q[2] * self.Q[3]
		self.q3q3 = self.Q[3] * self.Q[3]
	def quatern2ReverseRotMat(self):

		rotmat = [[0,0,0],[0,0,0],[0,0,0]]
		
		rotmat[0][0] = 2*( self.q0q0 + self.q1q1 ) - 1
		rotmat[0][1] = 2*( self.q1q2 - self.q0q3 )
		rotmat[0][2] = 2*( self.q1q3 + self.q0q2 )
		rotmat[1][0] = 2*( self.q1q2 + self.q0q3)
		rotmat[1][1] = 2*( self.q0q0 + self.q2q2 ) - 1
		rotmat[1][2] = 2*( self.q2q3 - self.q0q1 )
		rotmat[2][0] = 2*( self.q1q3 - self.q0q2 )
		rotmat[2][1] = 2*( self.q2q3 + self.q0q1)
		rotmat[2][2] = 2*( self.q0q0 + self.q3q3 ) - 1

		return rotmat

	def quatern2euler(self):
		angles = [None] * 3
                
		angles[0] = math.atan2(2 * self.q1q2 + 2 * self.q0q3, -2 * self.q2q2 - 2 * self.q3q3 + 1) # yaw
		angles[1] = -1*math.asin(2 * self.q1q3 - 2 * self.q0q2) # pitch
		angles[2] = math.atan2(2 * self.q2q3 + 2 * self.q0q1, -2 * self.q1q1 - 2 * self.q2q2 + 1)

		angles[0] *= self.RAD2DEG
		angles[1] *= self.RAD2DEG
		angles[2] *= self.RAD2DEG

		angles[0] = angles[0]
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
		t3 = math.sin(angles[2]*0.5/self.RAD2DEG)#0		
		t4 = math.cos(angles[1]*0.5/self.RAD2DEG)#1
		t5 = math.sin(angles[1]*0.5/self.RAD2DEG)#0
		

#		Qtmp = self.Q

		self.Q[0] = t0 * t2 * t4 + t1 * t3 * t5 # t0*1*1 + t1*0*0 = t1
		self.Q[1] = t0 * t3 * t4 - t1 * t2 * t5 # t0*0*1 - t1*t2*0 = 0
		self.Q[2] = t0 * t2 * t5 + t1 * t3 * t4 # t0*1*0 + t1*0*1 = 0
		self.Q[3] = (t1 * t2 * t4 - t0 * t3 * t5) # t1*1*1 - t0*0*0 = t1


		recipNorm = 1/math.sqrt(self.Q[0] * self.Q[0] + self.Q[1] * self.Q[1] + self.Q[2] * self.Q[2] + self.Q[3] * self.Q[3]);
		self.Q[0] *= recipNorm;
		self.Q[1] *= recipNorm;
		self.Q[2] *= recipNorm;
		self.Q[3] *= recipNorm

		self.q0q0 = self.Q[0] * self.Q[0]
		self.q0q1 = self.Q[0] * self.Q[1]
		self.q0q2 = self.Q[0] * self.Q[2]
		self.q0q3 = self.Q[0] * self.Q[3]
		self.q1q1 = self.Q[1] * self.Q[1]
		self.q1q2 = self.Q[1] * self.Q[2]
		self.q1q3 = self.Q[1] * self.Q[3]
		self.q2q2 = self.Q[2] * self.Q[2]
		self.q2q3 = self.Q[2] * self.Q[3]
		self.q3q3 = self.Q[3] * self.Q[3]
		
#		print (Qtmp,self.Q)
		