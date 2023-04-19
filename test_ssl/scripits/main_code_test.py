#! /usr/bin/env python3

import numpy as np
from math import atan2
from time import sleep
from threading import *
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math
import random

#inicializar valores

ball = Pose()
p_ball=(0,0)
heading=0
#bola_robot = coordenada da bola
#euclides = math.sqrt((p_ball[0]-robot_b[0].pose.x)**2 + (p_ball[1]-robot_b[0].pose.y)**2)

robot_b = {
    0: SSL_DetectionRobot(),
    1: SSL_DetectionRobot(),
    2: SSL_DetectionRobot(),
    3: SSL_DetectionRobot(),
    4: SSL_DetectionRobot()
}

robot_y = {
    0: SSL_DetectionRobot(),
    1: SSL_DetectionRobot(),
    2: SSL_DetectionRobot(),
    3: SSL_DetectionRobot(),
    4: SSL_DetectionRobot()
}

dist = { #vector de distancia azul con pelota
    0: (0,0), 
    1: (0,0),
    2: (0,0),
    3: (0,0),
    4: (0,0)
}

dist_mod = { #modulo de distancia azul-pelota
    0: 0, 
    1: 0,
    2: 0,
    3: 0,
    4: 0
}
    
ang = { #angulo de distancia azul-pelota
    0: 0, 
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

dist_by = { #vector de distancia azul-amarillos
    0: [(0,0), (0,0), (0,0), (0,0), (0,0)], 
    1: [(0,0), (0,0), (0,0), (0,0), (0,0)],
    2: [(0,0), (0,0), (0,0), (0,0), (0,0)],
    3: [(0,0), (0,0), (0,0), (0,0), (0,0)],
    4: [(0,0), (0,0), (0,0), (0,0), (0,0)]
}

min_dist = { #menor distancia arco-robot
    0: 0, 
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

min_vec_dist ={ #menor vector distancia arco-robot
    0: (0,0), 
    1: (0,0),
    2: (0,0),
    3: (0,0),
    4: (0,0)
}

ang_arc = { #angulo arco-pelota de distancia minima
    0: 0, 
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

ssl_msg = {
        0:SSL(),
        1:SSL(),
        2:SSL(),
        3:SSL(),
        4:SSL()
    }

############### DATOS ####################

def data_set(data): #recibo datos de r_azul, r_amarillo y pelota
                
    for i in range(0, len(data.robots_blue)):
        id_robots = data.robots_blue[i].robot_id
        if id_robots == 0:
            robot_b[0] = data.robots_blue[i]
        if id_robots == 1:
            robot_b[1] = data.robots_blue[i]
        if id_robots == 2:
            robot_b[2] = data.robots_blue[i]
        if id_robots == 3:
            robot_b[3] = data.robots_blue[i]
        if id_robots == 4:
            robot_b[4] = data.robots_blue[i]

    for i in range(0, len(data.robots_yellow)):
        id_robotsy = data.robots_yellow[i].robot_id
        if id_robotsy == 0:
            robot_y[0] = data.robots_yellow[i]
        if id_robotsy == 1:
            robot_y[1] = data.robots_yellow[i]
        if id_robotsy == 2:
            robot_y[2] = data.robots_yellow[i]
        if id_robotsy == 3:
            robot_y[3] = data.robots_yellow[i]
        if id_robotsy == 4:
            robot_y[4] = data.robots_yellow[i]
    
    global ball		
    ball = data.balls

def save_ball(): #salvo los datos de la pelota

    global p_ball

    try: 
        p_ball=((ball[0].x),(ball[0].y))
        dist_angle(p_ball[0], p_ball[1]) 
        #print('return')
        return(p_ball)

    except: 
        #print('except') 
        pass


######### funciones de cancha ###########

'''datos de la cancha


# Blue Area
# [-2000, 1000] -------------------- [-1000, 1000]
# |      0                              |   1
# |                                     |
# |                                     |
# |      3                              |    2
# [-2000, -1000] ------------------- [-1000, -1000]

# Field size
# [-2000, 2000]----------------[0, 2000]--------------------[2000, 2000]
# |      0                        1                              | 2
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |                               |                              |
# |      5                        4                              |  3
# [-2000, -2000]---------------[0,-2000]------------------[2000, -2000]
'''

def InArea(positionX, positionY): 
    if positionX > -2000 and positionX < 0:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False

def field(positionX, positionY): 
    if positionX > -2000 and positionX < 2000:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False

def defensePosition(positionX, positionY):
    if positionX > -2000 and positionX < 0:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False

####### cálculo de distancias ##########

def dist_angle(ball_x, ball_y): #azul-pelota

    for i in range (0,5):
        dist[i]= (ball_x-robot_b[i].x,ball_y-robot_b[i].y) #vector de distancia
        dist_mod[i]= math.sqrt((ball_x-robot_b[i].x)**2+(ball_y-robot_b[i].y)**2) #modulo de distancia
        ang[i]= math.atan2(dist[i][1], dist[i][0]) #angulo robot-pelota

def dist_robots(): #azul-amarillo

    for i in range (0,5):
        for j in range (0,5):
            dist_by[i]=math.sqrt((robot_y[j].x-robot_b[i].x)**2 + (robot_y[j].y-robot_b[i].y)**2)

def arc_robots(robotIndex): #arco-robot

#(1900,950)-----------(1900,-950) 
    global dist_arc_robot
    global ang_v2,ang_v1
    global ang_min

    dist_arc_robot = []
    vect_arc_robot = []
    ang_arc_robot = []

    
    ang_min = 0 #ANGULO DE LA DISTANCIA MINIMA

    for h in range(-950,1000,50):
        dist_arc_robot.append(math.sqrt((1900-robot_b[robotIndex].x)**2+(h-robot_b[robotIndex].y)**2)) #modulo distancia arco-robot
        
        vect_arc_robot.append(((1900-robot_b[robotIndex].x),(h-robot_b[robotIndex].y))) #vector distancia arco-robot
        
        if (1900-robot_b[robotIndex].x) != 0: #esto es porque no se pued dividir entre 0
            ang_arc_robot.append(math.atan2((h-robot_b[robotIndex].y),(1900-robot_b[robotIndex].x))) #angulo arco-robot
        
        else: ang_arc_robot.append(0)

    min_dist[robotIndex]=min(dist_arc_robot)

#angulo formado por robot, p1-arco y p2-arco

    vector_1 = vect_arc_robot[len(vect_arc_robot)-1]
    vector_2 = vect_arc_robot[0]

    ang_v1 = (math.atan2((vector_1[0]-robot_b[robotIndex].y),(vector_1[1]-robot_b[robotIndex].x)))
    ang_v2 = (math.atan2((vector_2[0]-robot_b[robotIndex].y),(vector_2[1]-robot_b[robotIndex].x)))



#ANGULO DE LA DISTANCIA MINIMA ----- hay que ver mejor esta parte xq hay conflicto

    if abs((h-robot_b[robotIndex].y)) < 0.02:
        ang_arc[robotIndex] = 0

    elif abs((1900-robot_b[robotIndex].x)) < 0.02:
        ang_arc[robotIndex] = (math.pi)/2 #aca tengo problema de orientacion de robot

    else:
        ang_arc[robotIndex] = ang_arc_robot[dist_arc_robot.index(min(dist_arc_robot))]

####### funciones de movimiento ######## 

def detect(robotIndex): #verifica que el robot este dentro de la cancha 

    if field(robot_b[robotIndex].x,robot_b[robotIndex].y) >= 1980:
        ssl_msg[robotIndex].cmd_vel.angular.z=0.0
        ssl_msg[robotIndex].cmd_vel.linear.x=-0.8
    if field(robot_b[robotIndex].x,robot_b[robotIndex].y) <= -1980:
        ssl_msg[robotIndex].cmd_vel.angular.z=0.0
        ssl_msg[robotIndex].cmd_vel.linear.x=0.8
        
    pub[robotIndex].publish(ssl_msg[robotIndex])	

def attack(robotIndex): #voy hacia la pelota
    global heading	

    heading = ang[robotIndex] - robot_b[robotIndex].orientation

    if heading > math.pi:
        heading -= 2 * math.pi

    elif heading < -math.pi:
        heading += 2 * math.pi

 
    if (-0.1) <= (heading) <= (0.1): 
        ssl_msg[robotIndex].cmd_vel.angular.z=0.0
        ssl_msg[robotIndex].cmd_vel.linear.x=0.5
        #ssl_msg[robotIndex].cmd_vel.linear.y=0.5
  
    elif heading <(-0.1):
        ssl_msg[robotIndex].cmd_vel.angular.z=-1.0
        ssl_msg[robotIndex].cmd_vel.linear.x=0.0
        #ssl_msg[robotIndex].cmd_vel.linear.y=-0.2
  
    else: 
        ssl_msg[robotIndex].cmd_vel.angular.z=1.0
        ssl_msg[robotIndex].cmd_vel.linear.x=0.0
        #ssl_msg[robotIndex].cmd_vel.linear.y=0.2

    pub[robotIndex].publish(ssl_msg[robotIndex])
 
###### pega bola e vira pro gol #######
def catch_ball(robotIndex):
    x_diff = p_ball[0] - robot_b[robotIndex].x
    y_diff = p_ball[1] - robot_b[robotIndex].y
    euclides = math.sqrt((x_diff)**2 + (y_diff)**2)
    if euclides <= 5:
        return robot_arco(robotIndex)
    else:
        return attack(robotIndex)

###### vira pro gol #######
def robot_arco(RobotIndex):
    x_diff = p_ball[0] - robot_b[RobotIndex].x
    y_diff = p_ball[1] - robot_b[RobotIndex].y
    euclides = math.sqrt((x_diff)**2 + (y_diff)**2)
    
    arco_x = 2000
    arco_y = list(range(-1000,1000,50))
    
    diff_x = arco_x - robot_b[RobotIndex].x
    diff_y = random.choice(arco_y) - robot_b[RobotIndex].y
    
    gama = math.atan2(diff_y, diff_x)
    err_orientation = gama - robot_b[RobotIndex].orientation

    
    if -0.1 <= err_orientation <= 0.1 and euclides <= 5:
        return kicker(RobotIndex)
    
    else: pass
        
    #goal(robotIndex)

    
###### chuta #######
def kicker(robotIndex):
    ssl_msg[robotIndex].kicker = True
     
    pub[robotIndex].publish(ssl_msg[robotIndex])
    


#### goalKepper ####
def KepperMovement(robotIndex):
    global keeper

    keeper = ang[robotIndex] - robot_b[robotIndex].orientation + math.pi

    if keeper > math.pi:
        keeper -= 2 * math.pi

    elif keeper < -math.pi:
        keeper += 2 * math.pi

    if abs(keeper) < 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.y = 1.2

    if keeper > 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = -0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    
    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

def stopMovement(robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    pub[robotIndex].publish(ssl_msg[robotIndex])
    pub[robotIndex].publish(ssl_msg[robotIndex])
    
def stopMovement(robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    pub[robotIndex].publish(ssl_msg[robotIndex])


def defend_ball(robotIndex):
    y = -500 if p_ball[1] < -500 else 500 if p_ball[1] > 500 else p_ball[1]
    diff_y = y - robot_b[robotIndex].y
    
    if abs(diff_y) > 50:
        ssl_msg[robotIndex].cmd_vel.linear.y = 1 if diff_y > 0 else -1
    else:
        ssl_msg[robotIndex].cmd_vel.linear.y = 0
    
    pub[robotIndex].publish(ssl_msg[robotIndex])
    
def defend_ball_defender(robotIndex):
    y = p_ball[1]
    diff_y = y - robot_b[robotIndex].y
    
    if abs(diff_y) > 50:
        ssl_msg[robotIndex].cmd_vel.linear.y = 1 if diff_y > 0 else -1
    else:
        ssl_msg[robotIndex].cmd_vel.linear.y = 0
    
    pub[robotIndex].publish(ssl_msg[robotIndex])
    
def MantainAngle(robotIndex):
    diff_y = 0 - robot_b[robotIndex].orientation
    
    if abs(diff_y) >= .1:
        ssl_msg[robotIndex].cmd_vel.angular.z = .1 if diff_y > 0 else -.1
    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0
    
    pub[robotIndex].publish(ssl_msg[robotIndex])
        
moving_to_goal = False
def MantainInLine(robotIndex):
    global moving_to_goal
    x = -1900
    diff_x = x - robot_b[robotIndex].x
    
    if abs(diff_x) > 7:
        moving_to_goal = True
        ssl_msg[robotIndex].cmd_vel.linear.x = .2 if diff_x > 0 else -.2
    elif moving_to_goal:
        moving_to_goal = False
        ssl_msg[robotIndex].cmd_vel.linear.x = 0
    
    pub[robotIndex].publish(ssl_msg[robotIndex])

def MantainInLine_Defender(robotIndex, at_x):
    x = at_x
    diff_x = x - robot_b[robotIndex].x
    
    if abs(diff_x) > 7:
        ssl_msg[robotIndex].cmd_vel.linear.x = .4 if diff_x > 0 else -.4
    elif moving_to_goal:
        ssl_msg[robotIndex].cmd_vel.linear.x = 0
        
goleiroBallDistance = lambda index: math.sqrt((p_ball[0]-robot_b[index].x)**2 + (p_ball[1]-robot_b[index].y)**2)

def move(id, pos : tuple, goal : tuple) -> None:
        # rospy.logerr(f"{self.goal_theta}, {self.theta}")
        KP_linear = 0.08
        KP_ang = 2.0
        delta = goal[2] - pos[2]
        if abs(delta) > np.pi:
            delta = (np.pi - delta) -1

        #Angular speed
        ssl_msg[id].cmd_vel.angular.z = KP_ang * delta

        #Linear speed
        d_x = goal[0] - pos[0]
        d_y = goal[1] - pos[1]
        theta2 = atan2(d_y, d_x)
        angle = theta2 - pos[2]
        dist = np.linalg.norm((d_x, d_y))
        speed = KP_linear * dist
        if speed >1.0:
            speed = 1.0
        v_x = np.cos(angle) * speed
        v_y = np.sin(angle) * speed
        ssl_msg[id].cmd_vel.linear.x=v_x
        ssl_msg[id].cmd_vel.linear.y=v_y
        pub[id].publish(ssl_msg[id])

        return

def attackBlue(id : int):
    if p_ball[1]>=0:
        move(id,(robot_b[id].x,robot_b[id].y,0),(p_ball[0],p_ball[1],0))
        if robot_b[id].x - p_ball[0]<10 and robot_b[id].y - p_ball[1]<10: 
            ssl_msg[id].kicker=True
            pub[id].publish(ssl_msg[id])
            ssl_msg[id].kicker=False
            pub[id].publish(ssl_msg[id])
            

def goalKeeper(robotIndex):
    MantainAngle(robotIndex)
    MantainInLine(robotIndex)
    
    # print(robot_b[robotIndex].y, p_ball)
    
    if InArea(p_ball[0], p_ball[1]):
        defend_ball(robotIndex)
        
        if goleiroBallDistance(robotIndex) < 400:
            kicker(robotIndex)
        
def AreaDefender1(positionX, positionY): 
    if positionX > -1800 and positionX < 100:
        if positionY > 500 and positionY < 1900:
            return True
        else:
            return False
    else:
        return False

def AreaDefender2(positionX, positionY): 
    if positionX > -1800 and positionX < 150:
        if positionY < -500 and positionY > -1900:
            return True
        else:
            return False
    else:
        return False
    
def AreaDefender3(positionX, positionY): 
    if positionX > -1000 and positionX < 100:
        if positionY > -1000 and positionY < 1000:
            return True
        else:
            return False
    else:
        return False
    
    
    
def BackDefender1(robotIndex, at_x, at_y):
    
    x = at_x
    y = at_y
    
    diff_x = x - robot_b[robotIndex].x
    diff_y = y - robot_b[robotIndex].y
        
    
    if abs(diff_x) > 7:
        ssl_msg[robotIndex].cmd_vel.linear.x = .2 if diff_x > 0 else -.2
    elif moving_to_goal:
        ssl_msg[robotIndex].cmd_vel.linear.x = 0
    

    if abs(diff_y) > 7:
        ssl_msg[robotIndex].cmd_vel.linear.y = .2 if diff_y > 0 else -.2
    elif moving_to_goal:
        ssl_msg[robotIndex].cmd_vel.linear.y = 0
    
def Defender1(robotIndex):
    MantainAngle(robotIndex)
    MantainInLine_Defender(robotIndex, -900)
    
    if AreaDefender1(p_ball[0], p_ball[1]):
        defend_ball_defender(robotIndex)
        
        if goleiroBallDistance(robotIndex) < 400:
            kicker(robotIndex)
    else:
        BackDefender1(robotIndex, -900, 1000)

def Defender2(robotIndex):
    MantainAngle(robotIndex)
    MantainInLine_Defender(robotIndex, -900)
    
    if AreaDefender2(p_ball[0], p_ball[1]):
        defend_ball_defender(robotIndex)
        
        if goleiroBallDistance(robotIndex) < 400:
            kicker(robotIndex)
    
    else: BackDefender1(robotIndex, -900, -1000)
    
def Defender3(robotIndex):
    MantainAngle(robotIndex)
    MantainInLine_Defender(robotIndex, -450)
    
    if AreaDefender3(p_ball[0], p_ball[1]):
        defend_ball_defender(robotIndex)
        
        if goleiroBallDistance(robotIndex) < 400:
            kicker(robotIndex)
    
    else: BackDefender1(robotIndex, -450, 0)
    


def goal(robotIndex): #voy hacia el arco
    
    global variable_1, variable_2
    
    if (dist_mod[robotIndex] < 150): #si estoy cerca de la pelota:
        arc_robots(robotIndex)

        variable_1 = ang_v1 - robot_b[robotIndex].orientation
        variable_2 = ang_v2 - robot_b[robotIndex].orientation

        if variable_1 > math.pi:
            variable_1 -= 2 * math.pi

        elif variable_1 < -math.pi:
            variable_1 += 2 * math.pi

        if variable_2 > math.pi:
            variable_2 -= 2 * math.pi

        elif variable_2 < -math.pi:
            variable_2 += 2 * math.pi

#aca en vez de estar alineada al arco verificar que estoy dentro del angulo
        ssl_msg[robotIndex].cmd_vel.angular.z=-0.2
        if -variable_1 <= robot_b[robotIndex].orientation <= variable_2: #si estoy dentro del angulo del arco
            ssl_msg[robotIndex].cmd_vel.angular.z=0.0
            ssl_msg[robotIndex].cmd_vel.linear.x=0.8
            print('dentro del angulo.......',variable_1,'........',variable_2,'.......', robot_b[robotIndex].orientation)

#Si tengo la pelota pero no estoy alineada al arco, QUE HAGO?????

        elif abs(robot_b[robotIndex].orientation) <(variable_1): #aca hay conflicto
            ssl_msg[robotIndex].cmd_vel.angular.z=-0.2
            ssl_msg[robotIndex].cmd_vel.linear.x=0.0
            print('por debajo del angulo.......',variable_1,'........',variable_2,'.......', robot_b[robotIndex].orientation)

        elif (robot_b[robotIndex].orientation) >(variable_2):
            ssl_msg[robotIndex].cmd_vel.angular.z=0.0
            ssl_msg[robotIndex].cmd_vel.linear.x=0.0
            print('por arriba del angulo.......',variable_1,'........',variable_2,'.......', robot_b[robotIndex].orientation)

        pub[robotIndex].publish(ssl_msg[robotIndex])

    else:
        pass
    
    #print(variable_1,'........',variable_2,'.......', robot_b[robotIndex].orientation)

if __name__=="__main__":
    rospy.init_node("detect", anonymous=False)
    
    sub= rospy.Subscriber("/vision", SSL_DetectionFrame, data_set)
    pub = {
        0:rospy.Publisher('/robot_blue_0/cmd', SSL, queue_size=10),
        1:rospy.Publisher('/robot_blue_1/cmd', SSL, queue_size=10),
        2:rospy.Publisher('/robot_blue_2/cmd', SSL, queue_size=10),
        3:rospy.Publisher('/robot_blue_3/cmd', SSL, queue_size=10),
        4:rospy.Publisher('/robot_blue_4/cmd', SSL, queue_size=10)
    }

    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        print(robot_b[1].y, p_ball)
        #sleep(.05)
        save_ball()
        #catch_ball(0)
        goalKeeper(1)
        Defender1(0)
        Defender2(2)
        Defender3(3)
        attackBlue(4)
        
        #### In field ####
        detect(0)
        detect(1)
        detect(2)
        detect(3)
        detect(4)
        
        
        #robot_arco(0)
          #print(robot_b[0])
        #attack(0)
        #robot_ball(0)
        #dist_angle()
        #arc_robots(0)
        #print(ang_v1,'------',-ang_v2,'------', robot_b[0].orientation)
        #goal(0)

#r.sleep(10)		
