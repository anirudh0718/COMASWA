# Controller which subscribes to tb3_/pos and sends twist command to tb3_/odom


from matplotlib.pyplot import axis
from numpy.lib.twodim_base import eye
import rospy
import numpy as np
from si_uni import create_si_to_uni_dynamics
from geometry_msgs.msg import Pose2D,Twist
from math import degrees
#from robres import calc_dot
import csv
from barrier import ebcf_control
from ROSrob import Robot_ROS
import pandas as pd
from FC_to_Goal import get_rob_gposes,get_pose,get_rob_rec_pos_ros,get_turned_rec,get_turn_orient,get_turn_orient_ros
from formge import gen2
BURGER_MAX_LIN_VEL = 1.12
BURGER_MAX_ANG_VEL = 1.4

si_to_uni_dyn = create_si_to_uni_dynamics()

dt = 0.2
d = 0.3

L3 = np.array([
    [3, -1, -1, -1],
    [-1, 3, -1, -1],
    [-1, -1, 3, -1],
    [-1, -1 , -1, 3]]) 

L2 = np.array([
    [1, -1],
    [-1, 1]])



# Tolerance
e1 = 0.02
e2 = 0.4
# Formation Distance for rectangle shape
Df_l = 1
Df_b = 0.5

ddiag_rec =np.sqrt(np.power(Df_l,2) + np.power(Df_b,2))

weights_rec3 = np.array([
    [0, Df_b,ddiag_rec,Df_l],
    [Df_b, 0, Df_l, ddiag_rec],
    [ddiag_rec, Df_l, 0, Df_b],
    [Df_l, ddiag_rec,Df_b, 0]
])

weights_rec2 = np.array([
    [0, Df_b],
    [Df_b, 0]
])


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, 0, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


const_obs = np.array([[10], [11.6]])
const_obs2 = np.array([[10], [11.8]])
obs = np.hstack((const_obs, const_obs2))

#poses = [r_pose,b_pose,g_pose]
subs =[]
pubs =[]
N = 4
Timer = 100
robot_names =['tb3_0','tb3_1','tb3_2','tb3_3']
class consesus_controller:
    def __init__(self):
        
        self.r_pose,self.b_pose,self.g_pose,self.o_pose = Pose2D(),Pose2D(),Pose2D(),Pose2D()
        print('STARTED CONSENSUS CONTROLLER')

        self.count = 0

        self.start1,self.start2 = gen2(np.array([-1.6,0.7]).reshape(2,),0.5)
        self.goal1,self.goal2 = gen2(np.array([1.6,0.2]).reshape(2,),0.5)
        self.start3,self.start4 = gen2(np.array([1.6,0.5]).reshape(2,),0.5)
        self.goal3,self.goal4 = gen2(np.array([-1.6,0.2]).reshape(2,),0.5)

        self.robots = []
        self.poses = [self.r_pose,self.b_pose,self.g_pose,self.o_pose]
        self.initform = False
        self.robots,self.robots1,self.robots2 = self.creat_2forms()
        self.cent = {'cent_F1':[],'cent_F2':[],'a':1,'b':0.5,'AF1':0,'AF2':0,'rel_velF1':[],'rel_velF2':[],'alpha_dotF1':0,'alpha_dotF2':0}
        for i in range(N):
            if i ==0:
                subs.append(rospy.Subscriber('/{}/pos'.format(robot_names[i]),Pose2D,self.red_pos_callback))
                #rospy.Subscriber('/{}/cmd_vel'.format(robot_names[i]),Twist,self.red_cmd_callback)
            if i ==1:
                subs.append(rospy.Subscriber('/{}/pos'.format(robot_names[i]),Pose2D,self.blue_pos_callback))
                #rospy.Subscriber('/{}/cmd_vel'.format(robot_names[i]),Twist,self.blue_cmd_callback)
            if i ==2:
                subs.append(rospy.Subscriber('/{}/pos'.format(robot_names[i]),Pose2D,self.green_pos_callback))
            if i ==3:
                subs.append(rospy.Subscriber('/{}/pos'.format(robot_names[i]),Pose2D,self.orange_pos_callback))
                #rospy.Subscriber('/{}/cmd_vel'.format(robot_names[i]),Twist,self.green_cmd_callback)
            pubs.append(rospy.Publisher('/{}/cmd_vel'.format(robot_names[i]),Twist, queue_size=1))



    def creat_2forms(self):
        robot1 = Robot_ROS(self.start1,self.goal1,0,1)
        robot2 = Robot_ROS(self.start2,self.goal2,1,1)
        robot3 = Robot_ROS(self.start3,self.goal3,2,1)
        robot4 = Robot_ROS(self.start4,self.goal4,3,1)

        robots = [robot1,robot2,robot3,robot4]
        robots1,robots2 = [robot1,robot2],[robot3,robot4]
        return robots,robots1,robots2

    def consesus(self):
        
            
        
        if self.count < Timer:
            self.send_vel()     
        else:
            print('STARTED CBF CONTROLLER')
            t1,t2 = self.formation_control()
            self.send_form2goal(t1,t2)

    def get_pose(self,robots):

        x = np.zeros((2, len(robots)))
        for i in range(len(robots)):
            x[:,i] = robots[i].state['q'][:2].reshape(2,)
        return x
    def get_form_cent(self,robots):

        pose = self.get_pose(robots)
        cent = np.mean(pose,axis=1)

        return cent

    def getformangle(self,poses):
        diff = poses[:,0] -poses[:,1]

        angle = np.arctan2(diff[1],diff[0])

        return degrees(angle)

    def calc_vel(self,pres,prev,dt):
        rel_vel = np.divide(np.subtract(pres,prev),dt)
        return rel_vel

    def calc_vel_c(self,pres,prev,dt):
        rel_vel = np.divide(np.subtract(pres,prev),dt)
        return rel_vel*(np.pi/180)

    def check_goal_reached(self,Robots):
        for i in range(N):
        #print(Robots[i].state['q'].shape[0])
            if np.allclose(Robots[i].state['q'],Robots[i].goal,rtol=0, atol=1e-07):
                return True
            else:
                return False

    

    def red_pos_callback(self,data):
        self.r_pose = data
        
    def blue_pos_callback(self,data):
        self.b_pose = data
       
    def green_pos_callback(self,data):
        self.g_pose = data
    def orange_pos_callback(self,data):
        self.o_pose = data
    
    def move2goal(self,arr_pos_red,arr_pos_blue,arr_pos_green,arr_pos_orange):
        #print('THis is where the robots start 1 2 3 4',self.start1,self.start2,self.start3,self.start4)
        x_dot = -1*np.array([[arr_pos_red - self.start1.reshape(2,1) ],[arr_pos_blue -self.start2.reshape(2,1)],[arr_pos_green -self.start3.reshape(2,1)],[arr_pos_orange - self.start4.reshape(2,1)]])
        x_dot = x_dot.reshape(8,1)
        #print(x_dot)
        return x_dot

    def update_pos(self):
        
        for i in range(N):
            
            # For 4 robot Foramtion
            self.robots[i].update_state(self.poses[i])

            



    def send_vel(self):
        
        arr_pos_red  = np.array([self.r_pose.x,self.r_pose.y]).reshape(2,1)

        arr_pos_blue  = np.array([self.b_pose.x,self.b_pose.y]).reshape(2,1)
        arr_pos_green  = np.array([self.g_pose.x,self.g_pose.y]).reshape(2,1)
        arr_pos_orange  = np.array([self.o_pose.x,self.o_pose.y]).reshape(2,1)


        pos_red  = np.array([self.r_pose.x,self.r_pose.y,self.r_pose.theta]).reshape(3,1)
        pos_blue  = np.array([self.b_pose.x,self.b_pose.y,self.b_pose.theta]).reshape(3,1)
        pos_green  = np.array([self.g_pose.x,self.g_pose.y,self.g_pose.theta]).reshape(3,1)
        pos_orange  = np.array([self.o_pose.x,self.o_pose.y,self.o_pose.theta]).reshape(3,1)


        
        X = np.array([[arr_pos_red],[arr_pos_blue],[arr_pos_green],[arr_pos_orange]])


        
        t_x = X.reshape(8,1)
        
        

        t_x_dot = self.move2goal(arr_pos_red,arr_pos_blue,arr_pos_green,arr_pos_orange)
        #print(t_x_dot)

                

        r_u = np.array([t_x_dot[0],t_x_dot[1]])
        b_u = np.array([t_x_dot[2],t_x_dot[3]])
        g_u = np.array([t_x_dot[4],t_x_dot[5]])
        o_u = np.array([t_x_dot[6],t_x_dot[7]])



        du_r = si_to_uni_dyn(r_u,pos_red)
        du_b = si_to_uni_dyn(b_u,pos_blue)
        du_g = si_to_uni_dyn(g_u,pos_green)
        du_o = si_to_uni_dyn(o_u,pos_orange)

        
        self.send_twist(du_r,du_b,du_g,du_o)
        self.count = self.count + 1

    def formation_control(self):
        dot1 = np.zeros((2,1,2,1))
        dot2 = np.zeros((2,1,2,1))
        self.cent['cent_F1'] = self.get_form_cent(self.robots1)
        self.cent['cent_F2'] = self.get_form_cent(self.robots2)

        self.cent['AF1'] = self.getformangle(self.get_pose(self.robots1))
        self.cent['AF2'] = self.getformangle(self.get_pose(self.robots2))

        for i in range(2):
            #print(self.robots[i].robot_step(obs,self.robots,i,i,7,L3,weights_rec3,e1,np.array([10,10]),0,[0,0],0))
            dot1[i] = self.robots1[i].robot_step(obs,self.robots1,i,i,7,L2,weights_rec2,e1,self.cent['cent_F2'],self.cent['AF2'],self.cent['rel_velF2'],self.cent['alpha_dotF2']).reshape(2,1)
            dot2[i] = self.robots2[i].robot_step(obs,self.robots2,i,i,7,L2,weights_rec2,e1,self.cent['cent_F1'],self.cent['AF1'],self.cent['rel_velF1'],self.cent['alpha_dotF1']).reshape(2,1)
                
            

                #print( 'These are centroids velocities',cent['rel_velF1'], cent['rel_velF2'])


                #print((get_form_cent(rbts),cent['AF1']))

            
        #self.update_pos()
        return dot1,dot2


    def send_form2goal(self,t_x_dot1,t_x_dot2):

        t_x_dot = np.concatenate((t_x_dot1,t_x_dot2),axis= 0)

        pos_red  = np.array([self.r_pose.x,self.r_pose.y,self.r_pose.theta]).reshape(3,1)
        pos_blue  = np.array([self.b_pose.x,self.b_pose.y,self.b_pose.theta]).reshape(3,1)
        pos_green  = np.array([self.g_pose.x,self.g_pose.y,self.g_pose.theta]).reshape(3,1)
        pos_orange  = np.array([self.o_pose.x,self.o_pose.y,self.o_pose.theta]).reshape(3,1)

        t_x_dot =t_x_dot.reshape(8,1)
        #print(t_x_dot)

        r_u = np.array([t_x_dot[0],t_x_dot[1]])
        b_u = np.array([t_x_dot[2],t_x_dot[3]])
        g_u = np.array([t_x_dot[4],t_x_dot[5]])
        o_u = np.array([t_x_dot[6],t_x_dot[7]])



        du_r = si_to_uni_dyn(r_u,pos_red)
        du_b = si_to_uni_dyn(b_u,pos_blue)
        du_g = si_to_uni_dyn(g_u,pos_green)
        du_o = si_to_uni_dyn(o_u,pos_orange)

        
        self.send_twist(du_r,du_b,du_g,du_o)
        self.poses = [self.r_pose,self.b_pose,self.g_pose,self.o_pose]
        self.update_pos()
        self.cent['rel_velF1'] = self.calc_vel(self.get_form_cent( self.robots1),self.cent['cent_F1'],0.01)
        self.cent['rel_velF2'] = self.calc_vel(self.get_form_cent( self.robots2),self.cent['cent_F2'],0.01)
        self.cent['alpha_dotF1'] = self.calc_vel_c(self.getformangle(self.get_pose(self.robots1)),self.cent['AF1'],0.01)
        self.cent['alpha_dotF2'] = self.calc_vel_c(self.getformangle(self.get_pose(self.robots2)),self.cent['AF2'],0.01)




    def send_twist(self,r_t,b_t,g_t,o_t): #g_t):


        r_vel = self.assign_vel(r_t)
        b_vel = self.assign_vel(b_t)
        g_vel = self.assign_vel(g_t)
        o_vel = self.assign_vel(o_t)

        vel_com =[r_vel,b_vel ,g_vel,o_vel]

        for i in range(N):
            pubs[i].publish(vel_com[i])


    def assign_vel(self,du):
        vel = Twist()
        vel.linear.x = checkLinearLimitVelocity(du[0])
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = checkAngularLimitVelocity(du[1])

        return vel


def si_unicycle(u,pos):

        dxu = si_to_uni_dyn(u,pos)
        return dxu

def write_data():
    names = ['red_bot.csv','blue_bot.csv','green_bot.csv']

    for i in range(3):
        with open(names[i], 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y", "Theta","x_dot","Linear_vel","Angular_vel"])

def clear_data():
    names = ['red.csv','blue.csv','green.csv']
    for i in range(3):
        with open(names[i], 'w', newline='') as file:
            file.close()

def cal_dist():
    pass



if __name__=='__main__':


    con = consesus_controller()
    try:
        rospy.init_node('consesus_cntroller', anonymous=True)
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        r = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            con.consesus()

            r.sleep()
    except rospy.ROSInterruptException:
        pass