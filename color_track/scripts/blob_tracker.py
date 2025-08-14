#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import cv2
import argparse
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

from dynamic_reconfigure.server import Server
from color_track.cfg import ColorTrackConfig


class CVImageSubscriber:
    def __init__(self, display_on):
        self._sub=rospy.Subscriber("/camera/color/image_rect_color/compressed", CompressedImage, self._cb)
        self._bridge = CvBridge()
        self._disp = display_on
        self._prev_time = 0

        self._lb = np.array([0,0,0])
        self._ub = np.array([255,255,255])
        self._opening_diam = 3
        self._closing_diam = 3
        self._reconfigure = Server(ColorTrackConfig, self._recon_cb)

        self._proc_times = [0 for i in range(60)]
        self._prev_elapsed = 0

        self._frame_pointer = 0

        self.recording = False
        self.data = []

    def _recon_cb(self,config, level):
        if config["opening_diam"]>=1:
            self._opening_diam = config["opening_diam"]
        else:
            self._opening_diam = 1
            config["opening_diam"]=1
        if config["closing_diam"]>=1:
            self._closing_diam = config["closing_diam"]
        else:
            self._closing_diam = 1
            config["closing_diam"]=1

        
            
        if config["h_min"]<=config["h_max"]:
            self._lb[0]=config["h_min"]
            self._ub[0]=config["h_max"]
        else:
            config["h_min"]=int(self._lb[0])
            config["h_max"]=int(self._ub[0])
            
        if config["s_min"]<=config["s_max"]:
            self._lb[1]=config["s_min"]
            self._ub[1]=config["s_max"]
        else:
            config["s_min"]=int(self._lb[1])
            config["s_max"]=int(self._ub[1])
            
        if config["v_min"]<=config["v_max"]:
            self._lb[2]=config["v_min"]
            self._ub[2]=config["v_max"]
        else:
            config["v_min"]=int(self._lb[2])
            config["v_max"]=int(self._ub[2])

        self._final_size_min = config["min_final_size"]
            
        return config
        
    def _cb(self,data):
        start = rospy.Time.now()
        time = data.header.stamp.to_nsec()
        frame_time = time-self._prev_time
        self._prev_time = time
        try:
            cv_image=self._bridge.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        center = self.process_image(self._lb, self._ub, cv_image, self._opening_diam, self._closing_diam, self._disp)

        history_len = 30
        self._frame_pointer = self._frame_pointer % len(self._proc_times)
        self._proc_times[self._frame_pointer] = self._prev_elapsed
        self._frame_pointer += 1

        time_per_frame = float(sum(self._proc_times))/float(len(self._proc_times))
        if time_per_frame != 0:
            rate = 1000000000/time_per_frame
        else:
            rate = "NA"
            
        print "Frame processing rate:", rate
        self._prev_elapsed = (rospy.Time.now()-start).to_nsec()
        if self.recording:
            self.data.append((frame_time/1000000000.0,center[0],center[1]))


    def start_recording(self):
        self.data = []
        self.recording = True

    def stop_recording(self):
        self.recording = False
        return self.data

    def process_image(self, hsv_lower, hsv_upper, cv_image, opening_diameter, closing_diameter, display_on=False):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv,hsv_lower,hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(opening_diameter,opening_diameter))
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(closing_diameter,closing_diameter))
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, kernel)
        
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                                    cv2.CHAIN_APPROX_TC89_L1)


        image_out = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        blobs = sorted(contours, key=lambda el: cv2.contourArea(el))

        centers = []
        for i in range(len(blobs)):
            blob = cv2.convexHull(blobs[i])
            if cv2.contourArea(blob)<self._final_size_min:
                continue
            
            M = cv2.moments(blob)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                centers.append(center)
            except ZeroDivisionError:
                pass
                
            color = np.uint8([[[(hsv_upper[0]+hsv_lower[0])/2,(hsv_upper[1]+hsv_lower[1])/2,(hsv_upper[2]+hsv_lower[2])/2]]])
        
            color = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)
            color = np.ndarray.tolist(color[0,0,:])
        
            cv2.circle(image_out, center, 4, (0,0,255), -1)
            cv2.drawContours(image_out, [blob], -1, color, 3)
        
        if self._disp:
            cv2.imshow("input",cv_image)
            cv2.imshow("output",image_out)
            cv2.waitKey(3)

        return centers[0]
        
if __name__=="__main__":
    rospy.init_node("blob_tracking")
    parser=argparse.ArgumentParser(description="Use opencv to get motion features from video")
    parser.add_argument('-d', '--display-video', help="Show the masked video on screen", action='store_true')
    #parser.add_argument('-n', '--name', help="Name of thing to be tracked (allows multiple nodes; this node will be called blob_tracking_[name])", action='store_true')
    args = parser.parse_known_args()[0]
    
    c = CVImageSubscriber(args.display_video)
    c.start_recording()
    rospy.sleep(5.0)
    print c.stop_recording()
    rospy.spin()
    
