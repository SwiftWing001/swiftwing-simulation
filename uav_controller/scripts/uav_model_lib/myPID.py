#! /usr/bin/env python3
#import rospy
import time

import matplotlib.pyplot as plot
import numpy as np

class PID():
    def __init__(self, dt, adjust_max, adjust_min, kp=1, ki=0.2, kd=0, p_up_full=99, p_down_full=-99, i_up_full=99, i_down_full=-99, d_up_full=99, d_down_full=-99,status_max = 9999., status_min = -9999.):
        self.adjust_max = adjust_max
        self.adjust_min = adjust_min
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0            #last time error
        self.all_err = 0        #total error
        self.time = dt          #time scale

        self.p_up_full = p_up_full
        self.p_down_full = p_down_full
        self.i_up_full = i_up_full
        self.i_down_full = i_down_full
        self.d_up_full = d_up_full
        self.d_down_full = d_down_full
        self.status_max = status_max
        self.status_min = status_min

    def calculate(self, expect, status, dt=-1):
        if dt <= 0:
            dt = self.time
        err = expect - status
        p_out = self.kp * err 				#p_out
        self.all_err += err*dt
        i_out = self.ki * self.all_err		#i_out
        d_out = self.kd * (err-self.err)/dt			#d_out
        self.err = err

        
        if p_out > self.p_up_full:
            p_out = self.p_up_full
        elif p_out < self.p_down_full:
            p_out = self.p_down_full
        if i_out > self.i_up_full:
            i_out = self.i_up_full
            if self.ki ==0:
                self.all_err = 0
            else:
                self.all_err = self.i_up_full/ self.ki
        elif i_out < self.i_down_full:
            i_out = self.i_down_full
            if self.ki == 0:
                self.all_err = 0
            else:
                self.all_err = self.i_down_full/ self.ki
        if d_out > self.d_up_full:
            d_out = self.d_up_full
        elif d_out < self.d_down_full:
            d_out = self.d_down_full

        adjust = p_out+i_out+d_out
        if (adjust > self.adjust_max):
            adjust = self.adjust_max
        elif (adjust < self.adjust_min):
            adjust = self.adjust_min
        #print(str(p_out)+'    '+str(i_out)+'    '+str(d_out))

        if status > self.status_max and adjust > 0:
            adjust = 0.
        elif status < self.status_min and adjust < 0.:
            adjust = 0.
        return adjust




"""
def main_fun():
	p_para = 0.1
	i_para = 0.1
	d_para = 0.1
	adjust_max = 10
	adjust_min = -10
	
	time_scale = np.linspace(1,100,1000)
	dt = time_scale[1]-time_scale[0]
	pid = PID(dt, adjust_max, adjust_min, p_para, i_para, d_para)

	status= 0
	expect = 10
	z=[]
	for i in time_scale:
		status = status+pid.calculate(expect, status)
		z.append(status)
	plot.figure(figsize=(8,6), dpi=80)
	plot.plot(time_scale, z, color="blue",linewidth=1.0, linestyle = "-")
	plot.show()


if __name__=='__main__':
	main_fun()
"""
