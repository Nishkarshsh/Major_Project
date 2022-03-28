from time import sleep
import numpy as np
import serial
import struct
import cv2
import time
import numpy as np
import math
import time


arduino = serial.Serial(port = "/dev/ttyUSB0", baudrate = 9600, timeout=1)
time.sleep(1)
a_1 = 9.40
a_2 = 14.00
a_3 = 13.4
a_4 = 10.7


frame = None


#Camera Variables
focus =  594.54172755
cx = 313.20811157
cy = 236.2389519
gain = 0.12
fovx= 56.7652484
fovy = 44.3568322
Z = 1

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
                if y > 0 and y < -90:
                    theta_4_possible.remove(y)

            theta_4_rad = min(theta_4_possible)

        elif config == "zero":
            

            theta_4_rad = theta_4_possible[0]

        return theta_4_rad

def get_aruco(frame, id):

    dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    param = cv2.aruco.DetectorParameters_create()
    corner, ids, rejected = cv2.aruco.detectMarkers(frame, dict, parameters = param)


    if len(corner) > 0:
        counter = -1
        for i in ids:
            counter += 1
            if i == id:
                corners = np.ravel(corner[counter])
                #Find center of the aruco
                center_x = ( float(corners[0]) + float(corners[2]) + float(corners[4]) + float(corners[6]) ) / 4
                center_y = ( float(corners[1]) + float(corners[3]) + float(corners[5]) + float(corners[7]) ) / 4
                list_x = [float(corners[0]),float(corners[2]),float(corners[4]),float(corners[6])]
                list_y = [float(corners[1]),float(corners[3]),float(corners[5]),float(corners[7])]
                list_x.sort()
                list_y.sort()
                bbox = (int(list_x[0]), int(list_y[0]), int(list_x[3] - list_x[0]), int(list_y[3]-list_y[0]))
        try:        
            return True, corners, bbox, center_x, center_y

        except:
            return False, None, None, None, None

    else:
        return False, None, None, None, None


def drawBox(frame, bbox):
    x,y,w,h = int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
    cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)


def pixel2image(u, v):
    x = (u-cx)/focus
    y = (v-cy)/focus
    return x ,y

def drawBox(frame, bbox):
    x,y,w,h = int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
    cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0), 2)

def get_x_y_distance(altitude_drone, x, y, fovx, fovy, resx, resy):
    ''' 
    Returns the distance in meters (x,y) 
    '''

    #changing x /  y  axis to be alligned with the origin
    x = x - frame.shape[1] / 2
    y = -(y - frame.shape[0] / 2)

    #Finding distance of drone in x axis
    length_x_axis = math.tan(math.radians(fovx/2))*altitude_drone
    pixel_to_length_x = 2*length_x_axis/resx
    length_x = x*pixel_to_length_x

    # Finding distance of drone in Y axis
    length_y_axis = math.tan(math.radians(fovy/2))*altitude_drone
    pixel_to_length_y = 2*length_y_axis/resy
    length_y = y*pixel_to_length_y
    return length_x, length_y

def moving_average(x, list_past):

    list_past.append(x)
    if len(list_past) > 10:
        list_past.pop(0)
    return sum(list_past)/10


moving_average_x = []
moving_average_y = []
tracking = False
tracker_locked = False

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

arm = HOTy(a_1, a_2, a_3, a_4)
t = 0.5
z = 7
x_past = 0
y_past = 0
while True:
    ret, frame = cap.read()
    if ret:
        detected, corners_det, bbox, center_x, center_y = get_aruco(frame, 0)
        #Creates KCF tracker 

        if tracker_locked:
            success, tracked_bbox =  tracker.update(frame)
            if success:
                #print('Tracked')
                drawBox(frame, tracked_bbox)
                tracking = True
                x_track,y_track,w,h = tracked_bbox
                target_x = tracked_bbox[0] + w/2
                target_y = tracked_bbox[1]+h/2
            if not success:
                #print('Tracking Lost')
                tracking = False
                tracker_locked = False

        if detected:
            print('locked ')
            tracker = cv2.TrackerKCF_create()
            tracker.init(frame, bbox)
            tracker_locked = True
            target_x = center_x
            target_y = center_y

        if detected or tracking:
                '''
                Calculating velocity to reduce the static error between the UAV and the target
                '''
                x1, y1 = pixel2image(target_x, target_y)
                #cv2.circle(frame, (int(corners[0]), int(corners[1])), 3,  (0, 255,0), -1)
                #cv2.circle(frame, (int(corners[2]), int(corners[3])), 3,  (0, 255,0), -1)
                #cv2.circle(frame, (int(corners[4]), int(corners[5])), 3,  (0, 255,0), -1)
                cv2.circle(frame, (int(target_x), int(target_y)), 3,  (0, 255,0), -1)

                interaction_matrix = np.array([
                    [-1/Z,   0 , x1/Z, x1*y1    ,  -(1+x1**2), y1 ],
                    [  0 , -1/Z, y1/Z, 1+y1**2,      -x1*y1  , -x1],
                    ], np.float32)
                inv_interaction_matrix = np.linalg.pinv(interaction_matrix)
                error = np.array([
                        [target_x-(frame.shape[1]/2)],
                        [target_y-(frame.shape[0]/2)],
                    ])
                velocity = -gain*np.matmul(inv_interaction_matrix, error)       
                print(round(float(-velocity[1]), 2),round(float(velocity[0]), 2))
                x = -velocity[1]
                y = velocity[2] 
                x_past = x
                y_past = y 
                config = 'up'
                theta_1_rad, theta_2_rad, theta_3_rad = arm.first_3Degrees(x, y, z, config)
                theta_3_rad = theta_3_rad + 1.57
                theta_1, theta_2, theta_3 = arm.deg(theta_1_rad), arm.deg(theta_2_rad), arm.deg(theta_3_rad)
                theta_4_rad = arm.fourth_Degree(config) - theta_2_rad - theta_3_rad + 1.57
                theta_4 = arm.deg(theta_4_rad) 

                thetas = [theta_1, theta_2, theta_3, theta_4]
                limbs = [arm.base, arm.shoulder, arm.elbow, arm.wrist]
                #print(limbs)
                #print(thetas)
                if arduino.isOpen() == False:
                    arduino.open()
                arduino.write(struct.pack('>B', (int(theta_1))))
                arduino.write(struct.pack('>B', (int(theta_2))))
                arduino.write(struct.pack('>B', (int(theta_3))))
                arduino.write(struct.pack('>B', (int(theta_4))))
                arduino.close()

        frame = cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 5, (255,255,0), -1)
        cv2.imshow('cam feed', frame)#
        key = cv2.waitKey(1) & 0xFF

        thetas = []
        limbs = []
        if key == 27:
            print('Mission Completed')
            break
    else:
        print('Frame lost')




    
