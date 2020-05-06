#!/usr/bin/env python

#Steer and drive keyboard controller
#Waypoint droping and saving ability

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

import sys, select, termios, tty

class PolarisKeyTeleop:

    msg = """
    Polaris Keyboard Controller
    ---------------------------
    Steer w/ Throttle: q / w / e
    Steer w/o Throttle: a / d
    Brake: s

    """

    keyboard_bindings = {
            'q': (1, 1),
            'w': (1, 0),
            'e': (1, -1),
            'a': (0, 1),
            's': (0, 0),
            'd': (0, -1),
            }

    sign = lambda self, x: x and (1, -1)[x<0]

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
    
        rospy.init_node('polaris_keyboard_controller')

        # waypoint_data_file = "/home/conner/gazebo_ws/learning_ws/src/polaris_ranger_sim/polaris_control/config/waypoints.txt";
        waypoint_data_file = rospy.get_param("~waypoint_export_address", "waypoints.txt");
        rospy.loginfo('%s waypoint export file address: \n %s', rospy.get_name(), waypoint_data_file)
        self.export_file = open(waypoint_data_file, 'w')
        self.dom_ = Odometry()

        steer_setpoint_topic = rospy.get_param("~steer_setpoint_topic", "steer_setpoint")
        drive_setpoint_topic = rospy.get_param("~drive_setpoint_topic", "drive_setpoint")
        steer_pub = rospy.Publisher(steer_setpoint_topic, Float64, queue_size=5)
        drive_pub = rospy.Publisher(drive_setpoint_topic, Float64, queue_size=5)
        odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        
        max_vel = rospy.get_param("~max_vel", 5.0)
        max_turn_angle = rospy.get_param("~max_steer", 0.4)

        x = 0
        th = 0
        status = 0
        count = 0
        acc = 0.1
        target_speed = 0
        break_speed = False
        control_speed = 0
        target_turn = 0
        control_turn = 0
        try:
            print(self.msg)
            print(self.vels(max_vel, max_turn_angle))
            while(1):
                #Listen for keys
                key = self.getKey()
                if key in self.keyboard_bindings.keys():
                    #Set coeff for drive (x) and turn (th)
                    x = self.keyboard_bindings[key][0]
                    th = self.keyboard_bindings[key][1]
                    if key == 's':
                        break_speed = True
                    else:
                        break_speed = False
                    count = 0
                #If spacebar then add waypoint
                elif key == ' ':
                    self.addWaypoint()
                #Command time outs, return coeffs to zero
                else:
                    count = count + 1
                    if count > 4:
                        x = 0
                        th = 0
                        break_speed = False
                    if (key == '\x03'):
                        break


                #Drive Velocity Smoother 
                #(control accelerations)
                target_speed = max_vel * x
                if target_speed > control_speed:
                    control_speed = min(target_speed, control_speed + max_vel/25.0)
                elif target_speed < control_speed:
                    control_speed = max(target_speed, control_speed - max_vel/5.0 )
                else:
                    control_speed = target_speed
    

                #Turn Velocity Smoother
                #(Restoring turns should be faster)
                target_turn = max_turn_angle * th
                if self.sign(target_turn) == self.sign(control_turn):
                    turn_vel = max_turn_angle / 75.0
                else:
                    turn_vel = max_turn_angle / 5.0

                if target_turn > control_turn:
                    control_turn = min(target_turn, control_turn + turn_vel)
                elif target_turn < control_turn:
                    control_turn = max(target_turn, control_turn - turn_vel)
                else:
                    control_turn = target_turn

                steer_msg = Float64()
                steer_msg.data = control_turn
                steer_pub.publish(steer_msg)

                drive_msg = Float64()
                #Consider break
                if not break_speed:
                    drive_msg.data = control_speed
                else:
                    drive_msg.data = 0
                drive_pub.publish(drive_msg)


        except Exception as e:
            print(e)

        finally:
            steer_msg = Float64()
            steer_msg.data = 0
            steer_pub.publish(steer_msg)
            
            drive_msg = Float64()
            drive_msg.data = 0
            drive_pub.publish(drive_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.export_file.close()


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self, speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def addWaypoint(self):
        x = self.odom_.pose.pose.position.x
        y = self.odom_.pose.pose.position.y
        rospy.loginfo('%s adding waypoint:\n(%f, %f)', rospy.get_name(), x, y)
        self.export_file.write(str(x))
        self.export_file.write('\n')
        self.export_file.write(str(y))
        self.export_file.write('\n')

    def odomCallback(self, data):
        self.odom_ = data 


if __name__=="__main__":
    t = PolarisKeyTeleop()

    