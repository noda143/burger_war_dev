#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program
by Takuya Yamaguhi.
'''

import rospy
import random
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from mask import mask_green
from mask import mask_red
from mask import mask_white

class RandomBot():
    def __init__(self, bot_name="NoName", use_camera=False):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        if use_camera:
            # for convert image topic to opencv obj
            self.gray_pix_num = 0
            self.mu_r = None
            self.mu_g = None
            self.mu_w = None
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.back_count = 0

    def calcTwist(self):


        if self.gray_pix_num < 500:    #敵が見えていない場合：適当に進む
　　　　　　if (self.mu_w['m00'] == 0): #追加
                th = 0.5
                print('反転')

            r_int = random.randint(-1,1)
            th = r_int

            #ぶつかって止まるので時々バック
            xr_int = random.randint(-1,9)
            if (xr_int < 0)or(self.back_count != 0):
                x = -0.2
                th = 1

                self.back_count += 1

                if self.back_count == 15:
                    self.back_count = 0

            else:
                x = 0.2


        elif self.gray_pix_num < 2500:   #敵が遠い場合：敵に対してまっすぐになるよう進む

            if (self.mu_r['m00'] != 0)and(self.mu_g['m00'] != 0):
                mr_x = int(self.mu_r["m10"]/self.mu_r['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ    
                mg_x = int(self.mu_g["m10"]/self.mu_g['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
            elif (self.mu_r['m00'] == 0):
                mg_x = int(self.mu_g["m10"]/self.mu_g['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
                mr_x = mg_x
            elif (self.mu_g['m00'] == 0):
                mr_x = int(self.mu_r["m10"]/self.mu_r['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
                mg_x = mr_x

            m_x = (mr_x + mg_x)/2.   
            #敵が自分から見て右にいるか左にいるか
            if m_x <= 320:  
                th = 0.5
            elif m_x > 320:
                th = -0.5

            x = 0.1
            print('--------')
            print('|   2   |')
            print('--------')

        else:          #敵が近い場合：退く
            if (self.mu_r['m00'] != 0)and(self.mu_g['m00'] != 0):
                mr_x = int(self.mu_r["m10"]/self.mu_r['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ    
                mg_x = int(self.mu_g["m10"]/self.mu_g['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
            elif (self.mu_r['m00'] == 0):
                mg_x = int(self.mu_g["m10"]/self.mu_g['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
                mr_x = mg_x
            elif (self.mu_g['m00'] == 0):
                mr_x = int(self.mu_r["m10"]/self.mu_r['m00']) #黒い物体の重心  左右どっちにいるかだけわかれば良いのでx座標のみ 
                mg_x = mr_x

            m_x = (mr_x + mg_x)/2.    
            #敵が自分から見て右にいるか左にいるか
            if m_x <= 320:  
                th = 0.5
            elif m_x > 320:
                th = -0.5

            x = -0.3

            print('--------')
            print('|   3   |')
            print('--------')
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(10) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(self.gray_pix_num)
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        masked_img_r = mask_red(self.img)
        masked_img_g = mask_green(self.img)
        #cv2.imshow("Image window", masked_img)
        #name_time = str(time.time())
        #cv2.imwrite(name_time+".png", self.img)
        #print(self.img)

        grayimg_r = cv2.cvtColor(masked_img_r, cv2.COLOR_BGR2GRAY) #グレースケール化
        grayimg_g = cv2.cvtColor(masked_img_g, cv2.COLOR_BGR2GRAY) #グレースケール化
        self.gray_pix_num = len(grayimg_r[grayimg_r<255])+len(grayimg_g[grayimg_g<255])   #黒い点の数を数える

        neg_grayimg_r = cv2.bitwise_not(grayimg_r)
        self.mu_r = cv2.moments(neg_grayimg_r, False)

        neg_grayimg_g = cv2.bitwise_not(grayimg_g)
        self.mu_g = cv2.moments(neg_grayimg_g, False)
        

        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('my_run')
    bot = RandomBot('TUIBI', use_camera=True)
    bot.strategy()

