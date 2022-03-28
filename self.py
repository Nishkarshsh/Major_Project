import math
print("please enter the coordinates of object in order x,y,z")
x= float(input())
y= float(input())
z= float(input())
print("please enter the dimensions of the object in order x,y,z")
xa = float(input())
ya = float(input())
za = float(input())
def inverseangles(x,y,z,xa,ya,za):
	a = 140
	b = 134
	c = 107
	def convert(a,b,c,d,e):
		a = a*57.32
		b = b*57.32
		c = c*57.32
		d = d*57.32
		e = e*57.32
		return a,b,c,d,e
	def dof(x1,y1,z1,a,b):
		l = x1*x1 + y1*y1 +z1*z1
		m = math.pow(l,1/2)
		alpha = math.asin(z1/m)
		breta = math.acos(-(b*b - a*a - m*m)/(2*a*m))
		theta2 = alpha + breta
		theta3 = math.acos(-(m*m - a*a - b*b)/(2*a*b))
		theta4 = 1.57 - theta2 - theta3 + 1.57
		return theta2,theta3,theta4
	if(za>xa and za>ya):
		theta1 = math.atan(y/x) 
		theta4 = 0
		x1 = x - a*math.cos(theta1)
		y1 = y - a*math.sin(theta1)
		theta2,theta3,theta5 = dof(x1,y1,za,a,b)
	elif(xa>ya):
		theta1 = math.atan(y/x)
		theta4 = 0
		z1 = z+a 
		theta2,theta3,theta5 = dof(x,y,z1,a,b)
	else:
		theta1 = math.atan(y/x)
		theta4 = 1.57
		z1 = z+a
		theta2,theta3,theta5 = dof(x,y,z1,a,b)
		theta5 = theta5 + 1.57 
	theta1,theta2,theta3,theta4,theta5 = convert(theta1,theta2,theta3,theta4,theta5)
	print (theta1,theta2,theta3,theta4,90-theta5)
	return theta1,theta2,theta3,theta4,theta5
inverseangles(x,y,z,xa,ya,za)


