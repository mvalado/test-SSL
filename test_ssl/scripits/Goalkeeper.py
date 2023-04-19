#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math


#inicializar valores

ball = Pose()
p_ball=(0,0)
heading=0

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

def recibir_datos(data): #recibo datos de r_azul, r_amarillo y pelota
				
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
    if positionX > -2000 and positionX < -1000:
        if positionY > -1000 and positionY < 1000:
            return True
        else:
            return False
    else:
        return False

def Cancha(positionX, positionY): 
    if positionX > -2000 and positionX < 2000:
        if positionY > -2000 and positionY < 2000:
            return True
        else:
            return False
    else:
        return False


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
    
def returnToGoal(robotIndex):
    x = -1900
    diff_x = x - robot_b[robotIndex].x
    if abs(diff_x) > 10:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = .1 if diff_x < 0 else -.1
	
    pub[robotIndex].publish(ssl_msg[robotIndex])


def goalKeeper(robotIndex):
    #if (InArea(robot_b[robotIndex].x, robot_b[robotIndex].y)):
    #    if (dist_mod[robotIndex] < 1000 and dist_mod[robotIndex] > 1):
    #        if (dist_mod[robotIndex] < 150):
    #            KepperMovement(robotIndex)
    #        else:
    #            KepperMovement(robotIndex)
    #    else:
    #        stopMovement(robotIndex)
    #else:
    returnToGoal(robotIndex)

if __name__=="__main__":
	rospy.init_node("detect", anonymous=False)
	
	sub= rospy.Subscriber("/vision", SSL_DetectionFrame, recibir_datos)
	pub = {
		0:rospy.Publisher('/robot_blue_0/cmd', SSL, queue_size=10),
		1:rospy.Publisher('/robot_blue_1/cmd', SSL, queue_size=10),
		2:rospy.Publisher('/robot_blue_2/cmd', SSL, queue_size=10),
		3:rospy.Publisher('/robot_blue_3/cmd', SSL, queue_size=10),
		4:rospy.Publisher('/robot_blue_4/cmd', SSL, queue_size=10)
	}

	r = rospy.Rate(100)