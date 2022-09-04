# import numpy as np
# import matplotlib.pyplot as plt

# ranges = 
# a=1
# b=0.1
# m=a-b*ranges
# h=np.add.reduceat(m, np.arange(0, len(m), 5))[0]
# hhh = np.hstack([h[-2:],h,h[:2]])
# smoothed = np.convolve([1,2,3,2,1], hhh)[4:-4]
# plt.scatter(smoothed,range(len(smoothed)))

        # if kfar<180 and knear>180 and 360-abs(kfar-knear)<abs(kfar-knear): ## age target tu valley nabashe ama valley beine 0-360 nabashe
        #         if signs[0]>0:
        #             classic=1
        #         if goal_angle< knear or goal_angle> kfar:
        #             valley=360 - (knear-kfar)
        #             if  valley> self.smax:
        #                 turn_angle = (knear+ self.smax/2)%360
        #                 rospy.loginfo("bigvalley")
        #             else:
        #                 turn_angle = (knear + (valley)/2)%360
        #                 rospy.loginfo("littlevalley")
        #         else: ## target tu valley bashe
        #             turn_angle = goal_angle
        #             rospy.loginfo("no obstacles in the way")
        #         if turn_angle<0:
        #             turn_angle+=360
                    
        # elif knear<180 and kfar>180  and 360-abs(kfar-knear)<abs(kfar-knear): ## age target tu valley nabashe ama valley beine 0-360 nabashe
        #         if signs[0]>0:
        #             classic=1
        #         if goal_angle> knear or goal_angle< kfar:
        #             valley=360 - (kfar-knear)
        #             if  valley> self.smax:
        #                 turn_angle = (knear-self.smax/2)
        #                 rospy.loginfo("bigvalley")
        #             else:
        #                 turn_angle = knear - (valley)/2
        #                 rospy.loginfo("littlevalley")
        #         else: ## target tu valley bashe
        #             turn_angle = goal_angle
        #             rospy.loginfo("no obstacles in the way")
        #         if turn_angle<0:
        #             turn_angle+=360


        # elif (goal_angle-kfar)*(goal_angle-knear)>0 or classic: ## age target tu valley nabashe
        #     rospy.loginfo("classic case")
        #     if abs(knear-kfar) > self.smax:
        #         turn_angle = knear+np.sign(kfar-knear)*self.smax/2
        #         rospy.loginfo(f"sign {np.sign(kfar-knear)}")
        #         rospy.loginfo("bigvalleyooooooooo")
        #     else:
        #         turn_angle = knear + (kfar-knear)/2
        #         rospy.loginfo("littlevalley")
        
            
        # else: ## target tu valley bashe
        #     turn_angle = goal_angle
        #     rospy.loginfo("no obstacles in the way")