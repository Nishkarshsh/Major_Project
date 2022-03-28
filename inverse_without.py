from time import sleep
import numpy as np
a_1 = 9.40
a_2 = 14.00
a_3 = 13.4
a_4 = 10.7

class HOTy():
    
    theta_1_rad = 0
    theta_2_rad = 0
    theta_3_rad = 0
    theta_4_rad = 0
    

    R0_1 = [[np.cos(theta_1_rad), 0,  np.sin(theta_1_rad)],
            [np.sin(theta_1_rad), 0, -np.cos(theta_1_rad)],
            [0                  , 1,                    0]]

    R1_2 = [[np.cos(theta_2_rad), -np.sin(theta_2_rad), 0],
            [np.sin(theta_2_rad),  np.cos(theta_2_rad), 0],
            [0                  ,  0                  , 1]]

    R2_3 = [[np.cos(theta_3_rad), -np.sin(theta_3_rad), 0],
            [np.sin(theta_3_rad),  np.cos(theta_3_rad), 0],
            [0                  ,  0                  , 1]]

    up = [[0,  -1,  0],
          [0,   0, -1],
          [1,   0,  0]]

    down = [[ 0,   1,  0],
            [ 0,   0, -1],
            [-1,   0,  0]]

    zero = [[1,  0,  0],
            [0,  0, -1],
            [0,  1,  0]]
    
    def __init__(self, a_1, a_2, a_3, a_4):

        
        self.a_1 = a_1
        self.a_2 = a_2
        self.a_3 = a_3
        self.a_4 = a_4

        self.base = 0  
        self.shoulder = 1  
        self.elbow = 2 
        self.wrist = 3 

        self.start_0 = 90
        self.start_1 = 135
        self.start_2 = -80
        self.start_3 = -80
   
    def deg(self, theta):
        
        return theta * (180/np.pi)

    def rad(self, theta):
        
        return theta * (np.pi/180)

    def first_3Degrees(self, x, y, z, config):
        
        
        theta_1_rad = np.arctan2(y, x) #1
        theta_1 = self.deg(theta_1_rad)

        r_1 = np.sqrt(y**2 + x**2) #2

        r_3 = z - self.a_1 #3

        r_2 = np.sqrt(r_3**2 + r_1**2) #4

        B_rad = np.arctan2(r_3, r_1) #5
        B = self.deg(B_rad)

        l = r_2 ** 2 - self.a_2 ** 2 - self.a_3 ** 2
        k = - 2 * self.a_2 * self.a_3 
        a_rad = np.arccos( l / k ) #6
        a = self.deg(a_rad)


        theta_3_rad = -(np.pi - a_rad) #7
        theta_3_rad__ = -1 * theta_3_rad #8
        theta_3 = self.deg(theta_3_rad)

        o = self.a_3 * np.sin(theta_3_rad__)
        p = (self.a_3 * np.cos(theta_3_rad__)) + self.a_2
        phi_rad = np.arctan2(o, p) #9
        phi = self.deg(phi_rad)


        theta_2_rad = phi_rad + B_rad #10
        theta_2 = self.deg(theta_2_rad)

        if theta_1_rad > np.pi:
            theta_1_rad = np.pi

        if theta_2_rad > np.pi:
            theta_2_rad = np.pi

        if theta_1_rad < 0:
            theta_1_rad = 0

        if theta_2_rad < 0:
            theta_2_rad = 0

        if theta_3_rad > np.pi/2:
            theta_3_rad = np.pi/2

        if theta_3_rad < -np.pi/2:
            theta_3_rad = -np.pi/2  
    

        return theta_1_rad, theta_2_rad, theta_3_rad
    
    def fourth_Degree(self, config):
        
        R0_2 = np.dot(self.R0_1, self.R1_2)
        R0_3 = np.dot(R0_2, self.R2_3)
        
        #R0_3 = R0_1 x R1_2 x R2_3 
        
        R0_3Inverse = np.linalg.inv(R0_3)
        
        if config == "up":
            R0_4 = self.up
        elif config == "down":
            R0_4 = self.down
        elif config == "zero":
            R0_4 = self.zero
        
        R3_4_use = np.dot(R0_3Inverse, R0_4)        
        theta_4_rad_1 = np.arccos(R3_4_use[0][0])
        theta_4_rad_2 = np.arcsin(-(R3_4_use[0][1]))
        theta_4_rad_3 = np.arcsin(R3_4_use[1][0])
        theta_4_rad_4 = np.arccos(R3_4_use[1][1])

        theta_4_possible = [theta_4_rad_1, theta_4_rad_2, theta_4_rad_3, theta_4_rad_4]
        
        if config == "up":
        
            for y in theta_4_possible:
                if y > 90 and y < 0:
                    theta_4_possible.remove(y)

            theta_4_rad = max(theta_4_possible)

        elif config == "down":

            for y in theta_4_possible:
                if y > 90 and y < 0:
                    theta_4_possible.remove(y)

            theta_4_rad = max(theta_4_possible)
            theta_4_rad = theta_4_rad - 1.57


        elif config == "zero":
            

            theta_4_rad = 1.57

        return theta_4_rad

if __name__ == '__main__':

    arm = HOTy(a_1, a_2, a_3, a_4)
    t = 0.5

    while True:

        x = float(input("Enter x-coordinate "))
        y = float(input("Enter y-coordinate "))
        z = float(input("Enter z-coordinate "))
        config = input("Set end-effector configuration ")
        theta_1_rad, theta_2_rad, theta_3_rad = arm.first_3Degrees(x, y, z, config)
        theta_3_rad = theta_3_rad + 1.57
        theta_1, theta_2, theta_3 = arm.deg(theta_1_rad), arm.deg(theta_2_rad), arm.deg(theta_3_rad)
        if theta_2_rad>1.57:
        	theta_4_rad = arm.fourth_Degree(config) -1.57 - theta_2_rad - theta_3_rad
        else:
        	theta_4_rad = arm.fourth_Degree(config) -1.57 - theta_2_rad - theta_3_rad +1.57	 
        if config == 'zero':
            theta_4_rad = 1.57
        theta_4 = arm.deg(theta_4_rad) 
        theta_3 = theta_3 + 90
        thetas = [theta_1, theta_2, theta_3, theta_4]
        limbs = [arm.base, arm.shoulder, arm.elbow, arm.wrist]
        print(limbs)
        print(thetas)

        sleep(2)

        thetas = []
        limbs = []
