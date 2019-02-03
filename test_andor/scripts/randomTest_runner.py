#! /usr/bin/env python

#import roslaunch_api_wrapper as raw
import signal
import sys
import time
import roslib
import rospy
import tf
import numpy
import rospy
from std_msgs.msg import String
import roslaunch
import subprocess# as child

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def signal_handler(signal, frame):
    print('Bye!')
    sys.exit(0)
    
def callback(data):
    rospy.loginfo("callbackLauncher: I heard %s",data.data)
    if(data.data=='RUN_TESTER'):
        print(bcolors.BOLD + bcolors.WARNING +"********** Run the and/or tester! **********"+ bcolors.ENDC)
        global child_tester
        global child_andor
        global count
        child_tester = subprocess.Popen(["rosrun","test_andor","random_test_andor"])
#         print("parent process")
#         print(child.poll())
#         rospy.loginfo('The PID of child: %d', child.pid)
#         print ("The PID of child:", child.pid)
        
    elif(data.data=='Test_DONE'):
        print(bcolors.BOLD + bcolors.WARNING +"********** Kill the and/or and tester! **********"+ bcolors.ENDC)
        time.sleep(0.1)
        child_tester.send_signal(signal.SIGINT)
        child_andor.send_signal(signal.SIGINT)
        
        print(bcolors.BOLD + bcolors.WARNING +"********************************************************************************"+ bcolors.ENDC)
        print(bcolors.BOLD + bcolors.WARNING +"********************************************************************************"+ bcolors.ENDC)
        print(bcolors.BOLD + bcolors.WARNING +"********************************************************************************"+ bcolors.ENDC)
        count=count+1
        
        if(count>=10):
            print "exit ..."
            sys.exit(0)
        
        time.sleep(1)
        child_andor= subprocess.Popen(["rosrun","andor","andor"])
#         child_tester = subprocess.Popen(["rosrun","test_andor","random_test_andor"])
        
        
    else:
        print(data.data)
        
#     print('pitt_runner process is alive!')

# def callbackKiller(data):
#     rospy.loginfo("callbackKiller: I heard %s",data.data)
#     if(data.data=='KILL_PITT'):
#         print("Kill the pitt!")
#         child.send_signal(signal.SIGINT)
#     print('pitt_runner process is alive!')

def main():
    
    global count
    count=0
    rospy.init_node('andor_randomTester')

    Sub = rospy.Subscriber('andorTester', String, callback)
    print('andor tester runner is alive!')
    global child_andor
    global child_tester
    child_andor= subprocess.Popen(["rosrun","andor","andor"])

#     rospy.spin()
    signal.signal(signal.SIGINT, signal_handler)

    print('Press Ctrl+C')
    signal.pause()

if __name__ == '__main__':
    main()
