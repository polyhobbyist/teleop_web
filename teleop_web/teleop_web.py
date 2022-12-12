#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import os
import sys
import tornado.ioloop
import tornado.web
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.netutil import bind_sockets
from tornado.web import Application

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# NOTE: Portions of this code mimic RosWebBridge server so that it is compatible on the same system
# see https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/rosbridge_server/scripts/rosbridge_websocket.py

def start_hook():
    IOLoop.instance().start()


def shutdown_hook():
    IOLoop.instance().stop()

class TeleopHandler(tornado.web.RequestHandler):
    def get(self):
        shareDir = get_package_share_directory('teleop_web')
        self.set_header("Cache-control", "no-cache")
        self.render(os.path.join(shareDir, "webroot/teleop_web.html"))

class GUIHandler(tornado.web.RequestHandler):
    def get(self):
        shareDir = get_package_share_directory('teleop_web')
        self.set_header("Cache-control", "no-cache")
        self.render(os.path.join(shareDir, "webroot/joystick_gui.json"))

class TeleopWebNode(Node):
    def __init__(self):
        super().__init__("teleop_web_node")
        shareDir = get_package_share_directory('teleop_web')
        self.get_logger().info('Working with share %s' % (shareDir))

        certfile = self.declare_parameter("certfile", "").value
        keyfile = self.declare_parameter("keyfile", "").value
        # if not set, set to None
        if certfile == "":
            certfile = None
        if keyfile == "":
            keyfile = None

        port = self.declare_parameter("port", 8088).value
        if "--port" in sys.argv:
            idx = sys.argv.index("--port") + 1
            if idx < len(sys.argv):
                port = int(sys.argv[idx])
            else:
                print("--port argument provided without a value.")
                sys.exit(-1)

        address = self.declare_parameter("address", "").value
        if "--address" in sys.argv:
            idx = sys.argv.index("--address") + 1
            if idx < len(sys.argv):
                address = sys.argv[idx]
            else:
                print("--address argument provided without a value.")
                sys.exit(-1)                

        handlers = [
            (r"/", TeleopHandler),
            (r"/joystick_gui.json", GUIHandler),
            
        ]

        application = Application(handlers)

        ssl_options = None
        if certfile is not None and keyfile is not None:
            ssl_options = {"certfile": certfile, "keyfile": keyfile}        

        sockets = tornado.netutil.bind_sockets(port, address)
        server = HTTPServer(application, ssl_options=ssl_options)
        server.add_sockets(sockets)



def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = TeleopWebNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_callback = PeriodicCallback(lambda: executor.spin_once(timeout_sec=0.01), 1)
    spin_callback.start()
    try:
        start_hook()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Exiting due to SIGINT")
    finally:
        shutdown_hook()  # shutdown hook to stop the server


if __name__ == "__main__":
    main()