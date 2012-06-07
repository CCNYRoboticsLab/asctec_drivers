#!/usr/bin/env python
# AscTec Autopilot Console Monitor
# Copyright (C) 2010, CCNY Robotics Lab
# William Morris <morris@ee.ccny.cuny.edu>
#
# http://robotics.ccny.cuny.edu
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

################################
# This code may not be pretty but is seems to work.
# Commands
# q = Quit
# r = Record Bag File
# f = Test flashing the screen
# b = Test the terminal bell

import roslib; roslib.load_manifest('asctec_mon')
import rospy
import curses
import subprocess
import os
import signal

from asctec_msgs.msg import LLStatus
from asctec_msgs.msg import IMUCalcData
from asctec_msgs.msg import GPSData

myscreen = curses.initscr()
curses.start_color()
curses.halfdelay(1)
curses.noecho()
curses.curs_set(0)
(maxx,maxy) = myscreen.getmaxyx()
llwin = curses.newwin(11, maxy, maxx-11, 0)
gpswin = curses.newwin(3, maxy, maxx-14, 0)
recwin = curses.newwin(3, maxy, maxx-17, 0)
imuwin = curses.newwin(maxx-17, maxy, 0, 0)
alarm = 0
alarm_count = 0
alarm_interval = 10
gps_lock = 1
imu_lock = 1
ll_lock = 1
rec_status = 0
rec_enable = 0
rec_cmd = ["rosbag", "record", "-a", "-o","asctec"]
rec_dir = str(os.environ['HOME'])+"/ros/bags"
rec_process = None
bag_name = None

def drawSignedVal(r,c,w,val,val_max,val_min,big):
    center = int(w/2)
    if (val > val_max):
        val = val_max
    if (val < val_min):
        val = val_min
    if big:
        # Draw Top
        imuwin.addch(r, c, curses.ACS_ULCORNER)
        for n in range(c+1, c+w):
            if (n == c+center):
                imuwin.addch(r, n, curses.ACS_TTEE)
            else:
                imuwin.addch(r, n, curses.ACS_HLINE)
        imuwin.addch(r, c+w, curses.ACS_URCORNER)
        r = r + 1

    # Draw Middle
    imuwin.addch(r, c, curses.ACS_VLINE)
    bar = int(float(val / val_max * center))
    if (bar == 0):
        imuwin.addch(r, c+center, curses.ACS_VLINE)
    elif (bar >= 0):
        imuwin.addstr(r, c+center, " "*bar, curses.color_pair(4))
        imuwin.addch(r, c+center, curses.ACS_VLINE,curses.color_pair(4))
    else:
        imuwin.addstr(r, c+center+bar+1, " "*(-1*bar), curses.color_pair(4))
        imuwin.addch(r, c+center, curses.ACS_VLINE,curses.color_pair(4))
    imuwin.addch(r, c+w, curses.ACS_VLINE)
    r = r + 1

    if big:
        # Draw Bottom
        imuwin.addch(r, c, curses.ACS_LLCORNER)
        for n in range(c+1, c+w):
            if (n == c+center):
                imuwin.addch(r, n, curses.ACS_BTEE)
            else:
                imuwin.addch(r, n, curses.ACS_HLINE)
        imuwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawBattery(r,c,w,battery_val):
    global alarm
    # Battery Settings
    # Taken from http://en.wikipedia.org/wiki/Lithium-ion_polymer_battery
    battery_max = 12.7   # Maximum Voltage
    battery_warn = 10.0  # Warning Voltage
    battery_min = 8.4    # Minimum Voltage

    # Draw Top
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        llwin.addch(r, n, curses.ACS_HLINE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    # Draw Middle
    llwin.addch(r, c, curses.ACS_VLINE)
    b = int((battery_val - battery_min)/(battery_max-battery_min)*w)
    if battery_val > battery_warn: 
        alarm = 0
        llwin.addstr(r, c+1, " " * b, curses.color_pair(4))
    else:
        alarm = 1
        llwin.addstr(r, c+1, " " * b, curses.color_pair(5))
    llwin.addch(r, c+w, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        llwin.addch(r, n, curses.ACS_HLINE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawStatusMode(r,c,w,data):
    # Draw Top
    size = int(w / 3)-1
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_TTEE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    size = size + 2
    pos = c+(size*0)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if (data.compass_enabled):
        llwin.addstr(r,pos+1,"Compass",curses.color_pair(3)|curses.A_BOLD)
    else:
        llwin.addstr(r,pos+1,"Compass",curses.color_pair(0))

    pos = c+(size*1)
    llwin.addch(r, pos, curses.ACS_VLINE)
    llwin.addstr(r,pos+1,"Flight Time: "+str(data.up_time)+" sec",curses.color_pair(0))

    pos = c+(size*2)
    llwin.addch(r, pos, curses.ACS_VLINE)
    llwin.addstr(r,pos+1,"CPU: "+str(data.cpu_load),curses.color_pair(0))

    pos = c+w
    llwin.addch(r, pos, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    size = int(w / 3)-1
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_BTEE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawFlightMode(r,c,w,flightMode):
    # Draw Top
    size = int(w / 5)-2
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        if (((n%(size+c))-2) or (n/size+c) > 6):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_TTEE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    size = size + 2
    # There are 5 Flight Modes but the bit index of the serial active
    # mode is currently unknown
    pos = c+(size*0)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b01111111)!=0b11111111):
        llwin.addstr(r,pos+1,"Emergency",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Emergency",curses.color_pair(2)|curses.A_BOLD)

    pos = c+(size*1)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11111101)!=0b11111111):
        llwin.addstr(r,pos+1,"Height Control",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Height Control",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*2)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11111011)!=0b11111111):
        llwin.addstr(r,pos+1,"GPS Mode",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"GPS Mode",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*3)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11011111)!=0b11111111):
        llwin.addstr(r,pos+1,"Serial Enable",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Serial Enable",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*4)
    llwin.addch(r, pos, curses.ACS_VLINE)
    # FIXME: This is probably the wrong bitmask
    if ((flightMode|0b10111111)!=0b11111111):
        llwin.addstr(r,pos+1,"Serial Active",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Serial Active",curses.color_pair(3)|curses.A_BOLD)
    pos = c+w
    llwin.addch(r, pos, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    size = int(w / 5)-2
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        if (((n%(size+c))-2) or (n/size+c) > 6):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_BTEE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def record_update():
    global rec_status
    global rec_cmd
    global rec_dir
    global rec_process
    global rec_enable
    global bag_name

    recwin.clear()
    if rec_status:
        recattr = curses.color_pair(2)
    else:
        recattr = curses.color_pair(0)
    recwin.attrset(recattr)
    (rec_maxx,rec_maxy) = recwin.getmaxyx()
    rec_maxy = rec_maxy - 2 # remove space for left and right border
    recwin.border(0)
    
    if rec_enable != rec_status:
        if rec_enable:
            rec_process = subprocess.Popen(rec_cmd, shell=False, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=rec_dir)
        else:
            bag_name = None
        rec_status = rec_enable
          
    if rec_status:
        if (bag_name == None or bag_name == ''):
            process = subprocess.Popen(['lsof -c record -Fn -- | grep active | cut -c2-'], shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=rec_dir)
            bag_name = process.stdout.readline().rstrip()
        recwin.addstr(0,2,"Flight Data Recording",curses.color_pair(2)|curses.A_BOLD)
        recwin.attrset(curses.color_pair(0))
        recwin.addstr(1, 2, "Filename: ")
        recwin.addstr(1,12,bag_name)
    else:
        if (rec_process != None):
            rec_process.send_signal(signal.SIGINT)
        recwin.addstr(0,2,"Flight Data Recorder",curses.color_pair(1)|curses.A_BOLD)
        recwin.attrset(curses.color_pair(0))
        recwin.addstr(1, 2, "Command: ")
        recwin.addstr(1,11,' '.join(rec_cmd))
    if rec_process != None:
      if rec_process.poll() == 0:
        rec_process = None
def gps_callback(data):
    global gps_lock
    gps_lock = 1
    gpswin.clear()
    (gps_maxx,gps_maxy) = imuwin.getmaxyx()
    gps_maxy = gps_maxy - 2 # remove space for left and right border
    gcol = 25
    gpswin.border(0)
    gpswin.addstr(0, 2, "GPS", curses.color_pair(1)|curses.A_BOLD)
    lat_val = float(data.latitude)/float(10**7)
    gpswin.addstr(1, 2, 'Lat: {0:+012.7f}'.format(lat_val))
    lon_val = float(data.longitude)/float(10**7)
    gpswin.addstr(1, 21, 'Lon: {0:+012.7f}'.format(lon_val))
    height_val = float(data.height)/1000.0
    gpswin.addstr(1, 40, 'Height: {0: 7.3f}m'.format(height_val))
    heading_val = float(data.heading)/1000.0
    gpswin.addstr(1, 58, 'Heading: {0: 7.3f}'.format(heading_val))
    gps_lock = 0

def imu_callback(data):
    global imu_lock
    imu_lock = 1
    imuwin.clear()
    (imu_maxx,imu_maxy) = imuwin.getmaxyx()
    imu_maxy = imu_maxy - 2 # remove space for left and right border
    gcol = 25
    imuwin.border(0)
    imuwin.addstr(0, 2, "AscTec Quadrotor Console Monitor", curses.color_pair(1)|curses.A_BOLD)

    pos = 1
    if (imu_maxx > 16):
        pos_inc = 3
        big = 1
    else:
        pos_inc = 1
        big = 0

    # Height Graph
    ################################
    height = float(data.height)/1000.0
    imuwin.addstr(pos+big, 2, 'Height:    {0: 8.3f}m'.format(height))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),height,10.0,-10.0,big)
    pos = pos + pos_inc
    
    # Roll Graph
    ################################
    roll = float(data.angle_roll)/1000.0
    imuwin.addstr(pos+big, 2, "Roll:      %+08.3f"%roll)
    imuwin.addch(pos+big, 21, curses.ACS_DEGREE)
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),roll,90.0,-90.0,big)
    pos = pos + pos_inc

    # Pitch Graph
    ################################
    pitch = float(data.angle_nick)/1000.0
    imuwin.addstr(pos+big, 2, "Pitch:     %+08.3f"%pitch)
    imuwin.addch(pos+big, 21, curses.ACS_DEGREE)
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),pitch,180.0,-180.0,big)
    pos = pos + pos_inc

    # Yaw Graph
    ################################
    yaw = float(data.angle_yaw)/1000.0 -180
    imuwin.addstr(pos+big, 2, "Fused Yaw: %+08.3f"%yaw)
    imuwin.addch(pos+big, 21, curses.ACS_DEGREE)
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),yaw,180.0,-180.0,big)
    pos = pos + pos_inc

    # Compass Graph
    ################################
    mag = float(data.mag_heading)/1000.0 -180
    imuwin.addstr(pos+big, 2, "Compass:   %+08.3f"%mag)
    imuwin.addch(pos+big, 21, curses.ACS_DEGREE)
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),mag,180.0,-180.0,big)
    pos = pos + pos_inc

    imu_lock = 0

def callback(data):
    global ll_lock
    ll_lock = 1
    llwin.clear()
    (maxx,maxy) = llwin.getmaxyx()
    maxy = maxy - 2 # remove space for left and right border
    gcol = 20
    llwin.border(0)
    llwin.addstr(0, 2, "Status", curses.color_pair(1)|curses.A_BOLD)

    # Battery Monitor
    ################################
    battery_val = float(data.battery_voltage_1)/1000.0
    llwin.addstr(2, 2, 'Battery: {0:.3f}V'.format(battery_val))
    drawBattery(1,gcol,maxy-(gcol+1),float(data.battery_voltage_1)/1000)

    # Flight Mode Monitor
    ################################
    drawFlightMode(4,2,maxy-3,data.flightMode)

    # Status Monitor
    ################################
    drawStatusMode(7,2,maxy-3,data)

    ll_lock = 0

def listener():
    global imuwin, maxx, maxy
    global alarm, alarm_count, alarm_interval
    global rec_enable

    rospy.init_node('asctec_monitor')
    rospy.Subscriber("asctec/LL_STATUS", LLStatus, callback)
    rospy.Subscriber("asctec/IMU_CALCDATA", IMUCalcData, imu_callback)
    rospy.Subscriber("asctec/GPS_DATA", GPSData, gps_callback)
    curses.init_pair(1, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_RED)
    r = rospy.Rate(10) # Hz
    (maxx,maxy) = myscreen.getmaxyx()
    while not rospy.is_shutdown():
        c = myscreen.getch()
        if c == ord('f'): curses.flash()
        elif c == ord('b'): curses.beep()
        elif c == ord('r'): rec_enable = not rec_enable
        elif c == ord('q'): break  # Exit the while()
        elif c == curses.KEY_HOME: x = y = 0
        (current_maxx,current_maxy) = myscreen.getmaxyx()
        if (current_maxx != maxx or current_maxy != maxy):
            (maxx,maxy) = myscreen.getmaxyx()
            gpswin.mvwin(maxx-14, 0)
            llwin.mvwin(maxx-11, 0)
            imuwin = curses.newwin(maxx-14, maxy, 0, 0)
            #imuwin.refresh()
            #llwin.refresh()
            #gpswin.refresh()
        if (alarm):
	    alarm_count = alarm_count + 1
            if (alarm_count == alarm_interval):
                alarm_count = 0
                curses.flash()
                curses.beep()
        if (not gps_lock):
            gpswin.refresh()
        if (not imu_lock):
            imuwin.refresh()
        if (not ll_lock):
            llwin.refresh()
        record_update()
        recwin.refresh()
        r.sleep()
    curses.nocbreak(); myscreen.keypad(0); curses.echo(); curses.curs_set(1)
    curses.endwin()

if __name__ == '__main__':
    listener()
