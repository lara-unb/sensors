#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2012
# Laboratório de Automação e Robótica, Universidade de Brasília (LARA/UnB)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, 
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, 
#    this list of conditions and the following disclaimer in the documentation 
#    and/or other materials provided with the distribution.
#  * Neither the name of Laboratório de Automação e Robótica or Universidade 
#    de Brasília nor the names of its contributors may be used to endorse or 
#    promote products derived from this software without specific prior 
#    written permission.
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

def cv_input():

def cv_output(width, height, name, d, k, r, p):
    body = (
    "# oST version 5.0 parameters\n"
    + "\n"
    + "\n"
    + "[image]\n"
    + "\n"
    + "width\n"
    + str(width) + "\n"
    + "\n"
    + "height\n"
    + str(height) + "\n"
    + "\n"
    + "[narrow_stereo/%s]" % name + "\n"
    + "\n"
    + "camera matrix\n"
    + " ".join(["%8f" % k[0,i] for i in range(3)]) + "\n"
    + " ".join(["%8f" % k[1,i] for i in range(3)]) + "\n"
    + " ".join(["%8f" % k[2,i] for i in range(3)]) + "\n"
    + "\n"
    + "distortion\n"
    + " ".join(["%8f" % d[i,0] for i in range(d.rows)]) + "\n"
    + "\n"
    + "rectification\n"
    + " ".join(["%8f" % r[0,i] for i in range(3)]) + "\n"
    + " ".join(["%8f" % r[1,i] for i in range(3)]) + "\n"
    + " ".join(["%8f" % r[2,i] for i in range(3)]) + "\n"
    + "\n"
    + "projection\n"
    + " ".join(["%8f" % p[0,i] for i in range(4)]) + "\n"
    + " ".join(["%8f" % p[1,i] for i in range(4)]) + "\n"
    + " ".join(["%8f" % p[2,i] for i in range(4)]) + "\n"
    + "\n")

    return body

def svs_header():
    header = (
    "# SVS Engine v 4.0 Stereo Camera Parameter File" + "\n"
    + "\n"
    + "\n"
    + "\n"
    + "[image]" + "\n"
    + "have_rect 1" + "\n"
    + "\n"
    + "\n"
    + "[stereo]" + "\n"
    + "frame 1.0" + "\n" 
    + "\n"
    + "\n"
    + "[external]"
    + "Tx -155.450087" 
    + "Ty -0.414297" 
    + "Tz -11.383758" 
    + "Rx -0.003320" 
    + "Ry -0.074937" 
    + "Rz 0.006018" 
    + "\n"
    + "\n")

    return header

def svs_output(width, height, name, d, k, r, p):
    body = (
    "[%s camera]" % name + "\n"
    + "pwidth %d" % width + "\n"
    + "pheight %d" % height + "\n"
    + "dpx 0.012000" + "\n"
    + "dpy 0.012000" + "\n"
    + "sx 1.000000" + "\n"
    + "Cx %6f" % k[0,2] + "\n"
    + "Cy %6f" % k[1,2] + "\n"
    + "f %6f"  % k[0,0] + "\n"
    + "fy %6f" % k[1,1] + "\n"
    + "alpha 0.000000" + "\n"
    + "kappa1 %6f" % d[0,0] + "\n"
    + "kappa2 %6f" % d[1,0] + "\n"
    + "kappa3 %6f" % d[4,0] + "\n"
    + "tau1 %6f" % d[2,0] + "\n"
    + "tau2 %6f" % d[3,0] + "\n"
    + "proj\n"
    + "  ".join(["%6ef" % p[0,i] for i in range(4)]) + "\n"
    + "  ".join(["%6ef" % p[1,i] for i in range(4)]) + "\n"
    + "  ".join(["%6ef" % p[2,i] for i in range(4)]) + "\n"
    + "rect\n"
    + " ".join(["%6ef" % r[0,i] for i in range(3)]) + "\n"
    + " ".join(["%6ef" % r[1,i] for i in range(3)]) + "\n"
    + " ".join(["%6ef" % r[2,i] for i in range(3)]) + "\n"
    + "\n
    + "\n")

    return body

def svs_footer():
    footer = (
    "[global]" + "\n"
    + "GTx 0.000000" + "\n" 
    + "GTy 0.000000" + "\n"  
    + "GTz 0.000000" + "\n" 
    + "GRx 0.000000" + "\n"  
    + "GRy 0.000000" + "\n"  
    + "GRz 0.000000" + "\n")

    return footer

def cv2svs(width, height, d_left, d_right, k_left, k_right, r_left, r_right, p_left, p_right):
    return svs_header() + svs_output(width, height, "left", d_left, k_left, r_left, p_left) + svs_output(width, height, "right", d_right, k_right, r_right, p_right) + svs_footer()

def svs2cv(width, height, d_left, d_right, k_left, k_right, r_left, r_right, p_left, p_right):
    return cv_output(width, height, "left", d_left, k_left, r_left, p_left) + cv_output(width, height, "right", d_right, k_right, r_right, p_right)

def main():
    in_file = calibration.opencv
    out_file = calibration-test.svs

    

import StringIO

main()
