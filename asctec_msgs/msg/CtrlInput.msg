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
# serial command (=Scientific Interface)
# Pitch input: -2047 .. 2047 (0 = neutral)
int16 pitch
# Roll input: -2047 .. 2047 (0 = neutral)
int16 roll
# R/C Stick input: -2047 .. 2047 (0 = neutral)
int16 yaw
# Collective: 0 .. 4095 (= 0 .. 100%)
int16 thrust
# control byte:
#    bit 0: pitch control enabled
#    bit 1: roll control enabled
#    bit 2: yaw control enabled
#    bit 3: thrust control enabled
#  These bits can be used to only enable one axis at a time and thus to control
#  the other axes manually. This usually helps a lot to set up and finetune
#  controllers for each axis seperately.
int16 ctrl
int16 chksum
