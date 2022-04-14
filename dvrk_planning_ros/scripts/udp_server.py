#!/usr/bin/env python3
import socket
import rospy
from geometry_msgs.msg import TransformStamped

localIP = "10.42.0.1"
localPort = 34567
bufferSize = 1024

msgFromServer = "hello udp client !!"
bytesToSend = str.encode(msgFromServer)
msgFromServer1 = "Just Catching up here"
bytesToSend1 = str.encode(msgFromServer1)
msgFromServer2 = "Lives in a brown brick house"
bytesToSend2 = str.encode(msgFromServer2)

#create Datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type= socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP,localPort))
print("UDP server is up and listening ")

if __name__ == '__main__':

    try:
        # ROS things
        pub = rospy.Publisher('servo_cp_follow', TransformStamped, queue_size=10)
        tf_stamped_msg = TransformStamped()
        tf_stamped_msg.header.frame_id = "world"
        tf_stamped_msg.child_frame_id = "input_controller"
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
            message = bytesAddressPair[0]
            address = bytesAddressPair[1]
            # clientMSG =  "message from client : {}".format(message)
            message_list = message.split()
            tf_stamped_msg.header.stamp = rospy.Time.now()
            tf_stamped_msg.transform.translation.x = float(message_list[0])
            tf_stamped_msg.transform.translation.y = float(message_list[1])
            tf_stamped_msg.transform.translation.z = float(message_list[2]) 
            print(tf_stamped_msg)

            tf_stamped_msg.transform.rotation.x = float(message_list[3])
            tf_stamped_msg.transform.rotation.y = float(message_list[4])
            tf_stamped_msg.transform.rotation.z = float(message_list[5])
            tf_stamped_msg.transform.rotation.w = float(message_list[6])
            pub.publish(tf_stamped_msg)

            #sending a reply to client
            UDPServerSocket.sendto(bytesToSend, address)
            UDPServerSocket.sendto(bytesToSend1, address)
            UDPServerSocket.sendto(bytesToSend2, address)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
