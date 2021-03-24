#!/usr/bin/env python
# license removed for brevity

#Importation des lirairies
import rospy
from std_msgs.msg import UInt16

#Creation de la fontion Talker qui envoie la vitesse a la carte Arduino
def talker():
    #On publie la vitesse dans la variable speed
    pub = rospy.Publisher('speed', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #Tant que le node n'est pas ferme la vitesse est demandee a l'utilisateur puis publiee dans le node
    while not rospy.is_shutdown():
        speed = int(input('speed : '))
        rospy.loginfo(speed)
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass