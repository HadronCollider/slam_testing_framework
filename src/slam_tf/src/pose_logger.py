#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import io
from tf.transformations import euler_from_quaternion
import json

if __name__ == '__main__':
    rospy.init_node('pose_logger', anonymous=True, log_level=rospy.INFO)
   
    target_frame = rospy.get_param('~target_frame')
    base_frame = rospy.get_param('~base_frame')
    output_file = rospy.get_param('~output_file')

    listener = tf.TransformListener()
    output = open(output_file, 'w+')
    rate = rospy.Rate(2)
    data = []
    try:
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform(target_frame, base_frame, rospy.Time(), rospy.Duration(10000))
            except tf.Exception:
                continue    		        		
            time = listener.getLatestCommonTime(target_frame, base_frame)
            (t, q) = listener.lookupTransform(target_frame, base_frame, time)
            output.write("%.6f %f %f 0.0 %f %f %f %f\n" % (time.secs +  time.nsecs/10.0 ** 9, t[0], t[1], q[0], q[1], q[2], q[3]))
            rate.sleep()            
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("pose_logger was interrupted")

    output.close()
