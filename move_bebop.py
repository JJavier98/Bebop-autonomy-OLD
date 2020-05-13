import roslib
try:
	roslib.load_manifest('msgs_to_cv2')
except:
	pass
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import termios
import sys, tty

def getch():
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()

takeoff_pub = rospy.Publisher('/bebop/takeoff',Empty, queue_size=10)
#takeoff_msgs = String()

land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
#land_msgs = String()

reset_pub = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
#reset_msgs = String()

move_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
move_msgs = Twist()
    
c = ''
rospy.init_node('bebop_flight', anonymous=True)

while c!='q':
	if c==' ':
		takeoff_pub.publish(Empty())
	elif c=='l':
		land_pub.publish(Empty())
	elif c=='e':
		print('emergency')
		reset_pub.publish(Empty())
	elif c=='h':
		print('help')
		
	c=getch()
	
	
	
	
	
