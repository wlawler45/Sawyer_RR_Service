import rospy
from intera_core_msgs.msg import DigitalOutputCommand
import time
import intera_io
from intera_interface.digital_io import DigitalIO

pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand, queue_size=10)
rospy.init_node('talker', anonymous=True)
msg=DigitalOutputCommand()
#msg.isInputOnly=False

msg.name="DO_0"
msg.value=True
#msg.state=1
pub.publish(msg)
head=DigitalIO("right_valve_1a")
head.state=False
tail=DigitalIO("right_valve_1b")
tail.state=True
#msg=DigitalOutputCommand()

#time.sleep(5)
#pub.publish(msg)


