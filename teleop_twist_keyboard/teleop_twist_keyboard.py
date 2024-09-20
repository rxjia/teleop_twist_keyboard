# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of the Willow Garage nor the names of its
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

import sys
import threading
import time

import geometry_msgs.msg
import rclpy
import numpy as np

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
    import select


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)
space key, k : force stop
anything else : stop smoothly

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

LOOP_RATE = 10.0
LOOP_PERIOD = 1.0 / LOOP_RATE

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        # tty.setraw(sys.stdin.fileno())
        # # sys.stdin.read() returns a string on Linux
        # key = sys.stdin.read(1)
        # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], LOOP_PERIOD)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return f'currently:\tspeed {speed}\tturn {turn} ({np.rad2deg(turn):.2f} deg/s) '


class Vel3DSmoother:
    """
    Smooth the 3D velocity based on acceleration and speed limits.
    """
    def __init__(self, cur_vel=np.zeros(3), acc=None, default_rate=10.0):
        self.acc = acc
        self.vel_control = cur_vel
        self.vel_target = cur_vel
        self.rate = default_rate

    def update(self, vel_target=None):
        if vel_target is None:
            vel_target = self.vel_target
        
        delta_vel = vel_target - self.vel_control
        delta_vel_norm = np.linalg.norm(delta_vel)
        if delta_vel_norm > 1e-5:
            delta_dir = delta_vel / delta_vel_norm
        else:
            delta_dir = np.zeros(3)
        if delta_vel_norm > self.acc:
            delta_vel = delta_dir * self.acc
        self.vel_control += delta_vel
        self.vel_target = vel_target
        return self.vel_control

    def reset(self):
        self.vel_control.fill(0.)
        self.vel_target.fill(0.)

class Vel6DSmoother:
    """
    Smooth the 6D velocity based on acceleration and speed limits [linear, rot].
    """
    def __init__(self, cur_vel=np.zeros(6), acc_linear=None, acc_rot=None, default_rate=10.0):
        self.acc_lin = acc_linear
        self.acc_rot = acc_rot
        self.vel_control = cur_vel
        self.vel_target = cur_vel
        self.t_prev = None
        self.rate = default_rate
    
    def update(self, vel_target=None, dt=None):
        if vel_target is None:
            vel_target = self.vel_target
        
        # dt = 1/self.rate
        t_now = time.perf_counter()
        if dt is None:
            if self.t_prev is None:
                self.t_prev = t_now
                dt = 1/self.rate
            else:
                dt = t_now - self.t_prev
        self.t_prev = t_now

        target_acc_lin = vel_target[0:3] - self.vel_control[0:3]
        target_acc_lin_norm = np.linalg.norm(target_acc_lin)
        if target_acc_lin_norm > 1e-5:
            target_acc_lin_dir = target_acc_lin / target_acc_lin_norm
        else:
            target_acc_lin_dir = np.zeros(3)
        if target_acc_lin_norm > self.acc_lin*dt:
            target_acc_lin = target_acc_lin_dir * self.acc_lin*dt

        target_acc_rot = vel_target[3:6] - self.vel_control[3:6]
        target_acc_rot_norm = np.linalg.norm(target_acc_rot)
        if target_acc_rot_norm > 1e-5:
            target_acc_rot_dir = target_acc_rot / target_acc_rot_norm
        else:
            target_acc_rot_dir = np.zeros(3)
        if target_acc_rot_norm > self.acc_rot*dt:
            target_acc_rot = target_acc_rot_dir * self.acc_rot*dt

        self.vel_control[0:3] += target_acc_lin
        self.vel_control[3:6] += target_acc_rot
        self.vel_target = vel_target
        return self.vel_control
    def reset(self):
        self.vel_control.fill(0.)
        self.vel_target.fill(0.)
    
def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    acc_lin = node.declare_parameter('acc_lin', 2.0).value
    acc_rot = node.declare_parameter('acc_rot', np.deg2rad(180)).value
    speed = node.declare_parameter('scale_linear', 0.5).value
    turn = node.declare_parameter('scale_angular', 1.0).value

    print(f"Params:\n stamped: {stamped}, frame_id: {frame_id},\n acc_lin: {acc_lin}, acc_rot: {acc_rot:.2f} ({np.rad2deg(acc_rot):.2f} deg/s), \n speed: {speed}, turn: {turn} ({np.rad2deg(turn):.2f} deg/s)")

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 5)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.
    y = 0.
    z = 0.
    th = 0.
    vel_smoother = Vel6DSmoother(acc_linear=acc_lin, acc_rot=acc_rot)

    status = 0.0
    count = 0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        # t0 = node.get_clock().now()
        while True:
            key = getKey(settings)
            # t1 = node.get_clock().now()
            # dt = (t1 - t0).nanoseconds / 1e9
            # t0 = t1
            # print(dt)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                count=0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count=0

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                vel_smoother.reset()
            else:
                count = count + 1
                if count > 4:
                    x = 0.
                    y=0.
                    z=0.
                    th = 0.
                if (key == '\x03'):
                    break
            
            vel_target = np.zeros(6)
            vel_target[0] = x*speed
            vel_target[1] = y*speed
            vel_target[2] = z*speed
            vel_target[5] = th*turn
            vel_control = vel_smoother.update(vel_target)

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = float(vel_control[0])
            twist.linear.y = float(vel_control[1])
            twist.linear.z = float(vel_control[2])
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(vel_control[5])
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
