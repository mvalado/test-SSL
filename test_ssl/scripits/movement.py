#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math
from functions import *
import time


ball = Pose()
robot0 = SSL_DetectionRobot()
robot1 = SSL_DetectionRobot()
robot2 = SSL_DetectionRobot()
robot3 = SSL_DetectionRobot()
robot4 = SSL_DetectionRobot()
robot = {
    0: SSL_DetectionRobot(),
    1: SSL_DetectionRobot(),
    2: SSL_DetectionRobot(),
    3: SSL_DetectionRobot(),
    4: SSL_DetectionRobot()
}
# inicializar valores
dist = {
    0: (0, 0),  # vector de distancia
    1: (0, 0),
    2: (0, 0),
    3: (0, 0),
    4: (0, 0)
}

dist_mod = {
    0: 0,  # modulo de distancia
    1: 0,
    2: 0,
    3: 0,
    4: 0
}

ang = {
    0: 0,  # angulo de distancia
    1: 0,
    2: 0,
    3: 0,
    4: 0
}
ssl_msg = {
    0: SSL(),
    1: SSL(),
    2: SSL(),
    3: SSL(),
    4: SSL()
}

p_ball = (0, 0)


def save_ball():

    global p_ball

    try:
        p_ball = ((ball[0].x), (ball[0].y))
        dist_angle(p_ball[0], p_ball[1])
        # print('return')
        return (p_ball)

    except:
        # print('except')
        pass


def dist_angle(x, y):
    global dist
    global dist_mod
    global ang
    dist = {
        0: (x-robot[0].x, y-robot[0].y),  # vector de distancia
        1: (x-robot[1].x, y-robot[1].y),
        2: (x-robot[2].x, y-robot[2].y),
        3: (x-robot[3].x, y-robot[3].y),
        4: (x-robot[4].x, y-robot[4].y)
    }

    dist_mod = {
        # modulo de distancia
        0: math.sqrt((x-robot[0].x)**2 + (y-robot[0].y)**2),
        1: math.sqrt((x-robot[1].x)**2 + (y-robot[1].y)**2),
        2: math.sqrt((x-robot[2].x)**2 + (y-robot[2].y)**2),
        3: math.sqrt((x-robot[3].x)**2 + (y-robot[3].y)**2),
        4: math.sqrt((x-robot[4].x)**2 + (y-robot[4].y)**2)
    }

    ang = {
        0: math.atan2(dist[0][1], dist[0][0]),  # angulo de distancia
        1: math.atan2(dist[1][1], dist[1][0]),
        2: math.atan2(dist[2][1], dist[2][0]),
        3: math.atan2(dist[3][1], dist[3][0]),
        4: math.atan2(dist[4][1], dist[4][0])
    }


def recibir_datos(data):

    for i in range(0, len(data.robots_blue)):
        id_robots = data.robots_blue[i].robot_id
        if id_robots == 0:
            robot[0] = data.robots_blue[i]
        if id_robots == 1:
            robot[1] = data.robots_blue[i]
        if id_robots == 2:
            robot[2] = data.robots_blue[i]
        if id_robots == 3:
            robot[3] = data.robots_blue[i]
        if id_robots == 4:
            robot[4] = data.robots_blue[i]

    global ball
    ball = data.balls


def movement(robotIndex):
    global heading

    heading = ang[robotIndex] - robot[robotIndex].orientation + math.pi

    if heading > math.pi:
        heading -= 2 * math.pi

    elif heading < -math.pi:
        heading += 2 * math.pi

    if abs(heading) < 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 1.2

    if heading > 0.1:
        ssl_msg[robotIndex].cmd_vel.angular.z = -0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    
    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def stopMovement(robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
    pub[robotIndex].publish(ssl_msg[robotIndex])


def shoot(robotIndex):
    ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
    ssl_msg[robotIndex].cmd_vel.linear.x = 1.0
    ssl_msg[robotIndex].kicker = True
    pub[robotIndex].publish(ssl_msg[robotIndex])


def returnToGoal(robotIndex):
    auxAngle = angleToGoal(robot[robotIndex].x, robot[robotIndex].y)
    aux = auxAngle - robot[robotIndex].orientation
    if abs(aux) < 0.05:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
        ssl_msg[robotIndex].cmd_vel.linear.x = 1.2

    else:
        ssl_msg[robotIndex].cmd_vel.angular.z = 0.8
        ssl_msg[robotIndex].cmd_vel.linear.x = 0.0

    pub[robotIndex].publish(ssl_msg[robotIndex])


def goalKeeper(robotIndex):
    if (InArea(robot[robotIndex].x, robot[robotIndex].y)):
        if (dist_mod[robotIndex] < 1000 and dist_mod[robotIndex] > 1):
            if (dist_mod[robotIndex] < 150):
                movement(robotIndex)
            else:
                movement(robotIndex)
        else:
            stopMovement(robotIndex)
    else:
        returnToGoal(robotIndex)


def returnToDefense(robotIndex, positionInField):
    auxAngle = angleToDefense(
        robot[robotIndex].x, robot[robotIndex].y, positionInField)
    aux = auxAngle - robot[robotIndex].orientation

    if (InDefensePosition(robot[robotIndex].x, robot[robotIndex].y, positionInField) == False):
        if abs(aux) < 0.05:
            ssl_msg[robotIndex].cmd_vel.angular.z = 0.0
            ssl_msg[robotIndex].cmd_vel.linear.x = 1.2
        else:
            ssl_msg[robotIndex].cmd_vel.angular.z = 0.8
            ssl_msg[robotIndex].cmd_vel.linear.x = 0.0
        pub[robotIndex].publish(ssl_msg[robotIndex])
    else:
        stopMovement(robotIndex)


def defense(robotIndex, positionInField):
    if (p_ball[0] < 0):
        movement(robotIndex)
        if (positionInField == 'left'):
            if (p_ball[1] > 0 and p_ball[1] < 1000):
                movement(robotIndex)
            else:
                returnToDefense(robotIndex, positionInField)
        if (positionInField == 'right'):
            if (p_ball[1] < 0 and p_ball[1] > -1000):
                movement(robotIndex)
            else:
                returnToDefense(robotIndex, positionInField)
    else:
        returnToDefense(robotIndex, positionInField)


def attack(robotIndex):
    if (dist_mod[robotIndex] < 150):
        shoot(robotIndex)
    else:
        movement(robotIndex)


if __name__ == "__main__":
    rospy.init_node("detect", anonymous=False)

    sub = rospy.Subscriber("/vision", SSL_DetectionFrame, recibir_datos)
    pub = {
        0: rospy.Publisher('/robot_blue_0/cmd', SSL, queue_size=10),
        1: rospy.Publisher('/robot_blue_1/cmd', SSL, queue_size=10),
        2: rospy.Publisher('/robot_blue_2/cmd', SSL, queue_size=10),
        3: rospy.Publisher('/robot_blue_3/cmd', SSL, queue_size=10),
        4: rospy.Publisher('/robot_blue_4/cmd', SSL, queue_size=10)
    }

    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        save_ball()
        # movement(0, 1.5, 0.5)
        goalKeeper(0)
        defense(1, "right")
        defense(2, "left")
        # attack(3)
        # attack(4)
