
import math
while True:
	x = input("enter the coordinate of x")
	y = input("enter the coordinate of y")
	z1 = input("enter the coordinate of z")
	z = z1 - 9.40 #length of shoulder 
	a = 14.00  #length of a1
	b = 13.40  #length of a2
	c = 10.70  #length of a3
	d = x*x + y*y +z*z
	e = math.pow(d,1/2)
	theta_1_nakli = math.acos(z/e)
	k = e - 10.70
	t = k + 10.70
	nx = k*x/t 
	ny = k*y/t 
	nz = k*z/t 
 	l = (a*a + b*b -k*k)/2/a/b
 	theta_2 = math.degrees(math.acos(l))
 	w= (a*a + k*k -b*b)/(2*a*k)
 	theta1 = math.acos(w)
	theta_1 = math.degrees(1.57 - theta_1_nakli + theta1) 
	theta_4 = math.degrees(1.57 + 3.14 - theta_2 - theta1)
	theta_0 = math.degrees(math.atan(y/x))
	theta_3 = 90
	print(theta_0)
	print(theta_1)
	print(theta_2)
	print(theta_3)
	print(theta_4)	


