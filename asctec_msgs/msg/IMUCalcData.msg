# Software License Agreement (BSD License)
#
# Copyright (c) 2010
#  William Morris <morris@ee.ccny.cuny.edu>
#  Ivan Dryanovski <ivan.dryanovski@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of CCNY Robotics Lab nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

Header header
# angles derived by integration of gyro_outputs, drift compensated by data fusion;
#-90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree

int32 angle_nick
int32 angle_roll
int32 angle_yaw

# angular velocities, raw values [16 bit], bias free, in 0.0154 degree/s (=> 64.8 = 1 degree/s)

int32 angvel_nick
int32 angvel_roll
int32 angvel_yaw

# acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g

int16 acc_x_calib
int16 acc_y_calib
int16 acc_z_calib

# horizontal / vertical accelerations: -10000..+10000 = -1g..+1g

int16 acc_x
int16 acc_y
int16 acc_z

# reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree

int32 acc_angle_nick
int32 acc_angle_roll

# total acceleration measured (10000 = 1g)

int32 acc_absolute_value

# magnetic field sensors output, offset free and scaled; units not determined, 
# as only the direction of the field vector is taken into account

int32 Hx
int32 Hy
int32 Hz

# compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree

int32 mag_heading

# pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown;
# used for short-term position stabilization

int32 speed_x
int32 speed_y
int32 speed_z

# height in mm (after data fusion)

int32 height

# diff. height in mm/s (after data fusion)

int32 dheight

# diff. height measured by the pressure sensor [mm/s]

int32 dheight_reference

# height measured by the pressure sensor [mm]

int32 height_reference

