#!/usr/bin/env python3
#
# =======================================================================
#   @file   webserver.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import os
import http.server
import socketserver


def main(args=None):
    rclpy.init(args=args)
    nh = Node('webserver')
    nh.declare_parameter('port')
    os.chdir(os.path.dirname(__file__) + '/../../../../share/pimouse_control2/contents')

    param = nh.get_parameter_or('port')
    if param.type_ == Parameter.Type.NOT_SET:
        port = 8000
    else:
        port = param.value

    handler = http.server.SimpleHTTPRequestHandler

    with socketserver.TCPServer(('', port), handler) as httpd:
        print('serving at port', port)
        httpd.serve_forever()

    nh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
