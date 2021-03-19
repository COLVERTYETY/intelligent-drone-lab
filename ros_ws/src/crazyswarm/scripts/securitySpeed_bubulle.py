################
#security speed bubulle
#original work from adem and txa
#added for loops to not repeat the same code 6 times


import math as m
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Point, Pose
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Float64MultiArray
from pycrazyswarm import *

a=1.35
b=0.85
c=1.1

########################################__creation des fonctions__########################################################
sigmoid = lambda v : 1/(1+m.exp(-v))                                       #fonction sigmoid entre 0-1 avec une evolution exponentielle proche de 0
distance_ellipsoid= lambda x,y,z : (x**2/a**2)+(y**2/b**2)+(z**2/c**2)     #distance avant sortie de l'arene normalise entre 0-1
closeness = lambda xa, xb, ya, yb, za, zb : ((xa-xb)**2+(ya-yb)**2+(za-zb)**2)**0.5                        #distance avant sortie de l'arene normalise entre 0-1

#TODO: ADEM- CHANGE ARENA BOUNDARIES AND ADD FUNCTION FOR DRONE COLLISIONS.

collision_distance = 0.6

x = {}
y = {}
z = {}
securitySpeed = {}

# all drones we are using 
#change everytime you add another drone
all_drones = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6']
#initialize positions for the begining
for drone in all_drones:
    x[drone]=0
    y[drone]=0
    z[drone]=0

    securitySpeed[drone] = 0

def cf_callback(data):

        

        #According to the drone position assign a value beetween 0-1 following a sigmoid function --> tend to 0 when the drone is close to the net
        cf = data.transforms[0]

        #get the position
        x[cf.child_frame_id]=cf.transform.translation.x
        y[cf.child_frame_id]=cf.transform.translation.y
        z[cf.child_frame_id]=cf.transform.translation.z

        #calculate the speed according to the position in the arena
#         securitySpeed[cf.child_frame_id] = sigmoid(18*((1-distance_ellipsoid(x[cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id]))-0.3))
        securitySpeed[cf.child_frame_id] = sigmoid(distance_ellipsoid(x[cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id]))

        #print for debug
        print(str(cf.child_frame_id) + ' ' + str(distance_ellipsoid(x[cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id])))

        #look at every other drones to slow down if to close
        for drone in all_drones:
            if drone != cf.child_frame_id:
                if closeness(x[cf.child_frame_id], y[cf.child_frame_id], z[cf.child_frame_id], x[drone], y[drone], z[drone]) < collision_distance:
                    print(str(cf.child_frame_id) + 'slowing down')
                    # securitySpeed[cf.child_frame_id] = 0.2
                    # securitySpeed = sigmoid(18*((1-distance_ellipsoidx([cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id]))-0.3))
#                     securitySpeed[cf.child_frame_id] = sigmoid(distance_ellipsoid(x[cf.child_frame_id],y[cf.child_frame_id],z[cf.child_frame_id]))
                    securitySpeed[cf.child_frame_id] = sigmoid(closeness(x[cf.child_frame_id], y[cf.child_frame_id], z[cf.child_frame_id], x[drone], y[drone], z[drone]))
                    print(securitySpeed[cf.child_frame_id])


    


        msg.data = securitySpeed[cf.child_frame_id]
        securitySpeed_publisher.publish(msg)



if __name__ == '__main__':
#     x = {}
#     y = {}
#     z = {}
#     securitySpeed = {}

    # all_drones = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6']
    for drone in all_drones:
        x[drone]=0
        y[drone]=0
        z[drone]=0

        securitySpeed[drone] = 0
    # global securitySpeed, msg, x1, x2, x3, y1, y2, y3, z1, z2, z3, x4, x5, x6, y4, y5, y6, z4, z5, z6

    # x1, x2, x3, y1, y2, y3, z1, z2, z3 = 1000, 100, 10, 1000, 100, 10, 1000, 100, 10
    # x4, x5, x6, y4, y5, y6, z4, z5, z6 = 1000, 100, 10, 1000, 100, 10, 1000, 100, 10
    # securitySpeed= [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    msg = Float64MultiArray()

    rospy.init_node('security', anonymous=True)

    securitySpeed_publisher = rospy.Publisher('/security_speed', Float64MultiArray, queue_size=10)
    cf2_subscriber = rospy.Subscriber('/tf', TFMessage, cf_callback)
    rospy.spin()
