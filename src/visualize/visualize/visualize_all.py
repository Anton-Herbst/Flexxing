#!/usr/bin/env python3

"""
* This file is to see if all transformation matrixes are indeed the ones i want.
* It should resemble the reals robot posture.
"""

import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vector
import matplotlib.pyplot as plt                                     # for visuals
import threading                                                    # so plt and ROS can work together

class Visualizer_All(Node):
    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('visualize_all')
        # * Parameters
        # how often the figure will be refreshed
        self.hz = 60
        self.color = ['r', 'm', 'c', 'b']
        # * ROS related
        # create a subscriber to get the vector from all magnetic sensors
        self.subscription_1  = self.create_subscription(Vector3, '/pc/mag_calib_1', self.mag_callback_1, 10)
        self.subscription_2  = self.create_subscription(Vector3, '/pc/mag_calib_2', self.mag_callback_2, 10)
        self.subscription_3  = self.create_subscription(Vector3, '/pc/mag_calib_3', self.mag_callback_3, 10)
        self.subscription_4  = self.create_subscription(Vector3, '/pc/mag_calib_4', self.mag_callback_4, 10)
        # also create a timer to refresh the plot
        self.planner = self.create_timer(1/self.hz, self.planner_callback)
        # * Visual related
        plt.ion()  # interactive mode ON
        plt.style.use('_mpl-gallery')
        # create a window 
        self.fig = plt.figure(figsize = (8,8), num="Grobe Pose des Roboters")
        # add an 3d plot in it
        self.axis = self.fig.add_subplot(projection='3d')
        # create a list of quivers (arrows/vectors) starting upright
        self.vector = [
            self.axis.quiver(0, 0, 0, 0, 0, 100, color=self.color[0], arrow_length_ratio=0.1),
            self.axis.quiver(0, 0, 100, 0, 0, 200, color=self.color[1], arrow_length_ratio=0.1),
            self.axis.quiver(0, 0, 200, 0, 0, 300, color=self.color[2], arrow_length_ratio=0.1),
            self.axis.quiver(0, 0, 300, 0, 0, 400, color=self.color[3], arrow_length_ratio=0.1) ]
        # create a list of Vector 3 to store the latest incoming data
        self.measured = [Vector3()] * 4
        # limit the axis
        limit = 100
        self.axis.set_xlim([-limit, limit])
        self.axis.set_ylim([-limit, limit])
        self.axis.set_zlim([-limit, limit])
        # label the axis
        self.axis.set_xlabel("X")
        self.axis.set_ylabel("Y")
        self.axis.set_zlabel("Z")

    # * callback of the timer
    def planner_callback(self) -> None:
        # starting quiver is done separately
        self.vector[0].remove()
        self.vector[0] = self.axis.quiver(0, 0, 0,
                            self.measured[0].x,
                            self.measured[0].y,
                            self.measured[0].z,
                            color=self.color[0],
                            arrow_length_ratio=0.1)
        #  tip-to-tail: each segment origin is the previous tip
        ox, oy, oz = self.measured[0].x, self.measured[0].y, self.measured[0].z
        # next three sensors all the way to the top follow the same procedure
        for i in range(1, 4):
            # erase the old one
            self.vector[i].remove()
            # create a new one with the prior ones end as this ones beginning
            self.vector[i] = self.axis.quiver(
                ox, oy, oz,
                ox+self.measured[i].x,
                oy+self.measured[i].y,
                oz+self.measured[i].z,
                color=self.color[i],
                arrow_length_ratio=0.1)
            # advance origin to tip of this segment
            ox += self.measured[i].x
            oy += self.measured[i].y
            oz += self.measured[i].z
        # after all quivers have been created redraw the figure
        self.fig.canvas.draw()

    # * callbacks of sensors
    # simply store the incoming msg
    def mag_callback_1(self, msg: Vector3) -> None: self.measured[0] = msg
    def mag_callback_2(self, msg: Vector3) -> None: self.measured[1] = msg
    def mag_callback_3(self, msg: Vector3) -> None: self.measured[2] = msg
    def mag_callback_4(self, msg: Vector3) -> None: self.measured[3] = msg
        
def main():
    rclpy.init()
    mynode = Visualizer_All()
    # spin the created note in a background tab 
    spin_thread = threading.Thread(target=rclpy.spin, args=(mynode,), daemon=True)
    spin_thread.start()
    # so that matplotlib can be in the foreground
    plt.show(block=True)
    # on exit clean up
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()