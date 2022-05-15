#!/usr/bin/python3
import rospy

from std_msgs.msg import Float32

omega = []
ros_time = []
node_state = 0
start_time = 0


#motor_effort_pub = rospy.Publisher("/motor_effort", Float32)

def my_callback(msg):
    global omega, ros_time, node_state

    w = msg.data

    #print('speed = {}'.format(w))

    if node_state == 1:
        omega.append(w)
        ros_time.append(rospy.get_time() - start_time)

    if node_state == 1 and omega[-1] > 600:
        node_state == 2
    

    if node_state == 2:
        for i in range(len(omega)):
            if i < 80:
                #print('dt {:02d} {} s {} rad/s'.format(int((ros_time[i+1]-ros_time[i])*1000), ros_time[i]- ros_time[0], omega[i]))
                print('dt = {:02d} ms, w = {:03.2f} rad/s'.format(int((ros_time[i+1]-ros_time[i])*1000), omega[i]))
        node_state = 3


    #print "Pos: ", pos, "vel: ", vel

def motor_effort_callback(msg):
    global node_state

    e = msg.data

    #print('effort = {}'.format(e))

    if e>0 and node_state == 0:
        node_state = 1

    if e == 0 and node_state == 1:
        node_state = 2



def my_node():
    global start_time
    rospy.init_node('motor_response_py') #initialzing the node with name "subscriber_py"

    rospy.Subscriber("/encoder_speed", Float32, my_callback, queue_size=10) 
    rospy.Subscriber("/motor_effort", Float32, motor_effort_callback, queue_size=10) 

    start_time = rospy.get_time()
    rospy.spin() 

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        pass