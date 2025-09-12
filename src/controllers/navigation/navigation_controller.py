from flask import jsonify
from flask import request
import os
from controllers.base_controller import BaseController


class NavigationController(BaseController):
    def __init__(self):
        self.name = "Navigation"
        self.main_sources = [
            "/opt/ros/noetic/setup.bash",
            os.path.expanduser("~/nav_stack/ws_livox/devel/setup.bash"),
            os.path.expanduser("~/nav_stack/ws_loc/devel/setup.bash"),
            os.path.expanduser("~/nav_stack/ws_nav/devel/setup.bash"),
        ]
        self.ros_bridge_sources = [
            "/opt/ros/noetic/setup.bash",
            "/opt/ros/foxy/setup.bash",
            os.path.expanduser("~/nav_stack/unitree_controller_ros2/setup.sh"),
        ]

        self.real_objects = [
            {
                "name": "Navigation",
                "type": "3d",
                "description": "Main Navigation display for the G1, contains, current location, live OccupancyGrid, goals, and planned paths. To set a goal via the web interface, hold shift and click on the desired location in the map.",
                "topics": [
                    {
                        "name": "pelvis",
                        "type": "Tf",
                    },
                    {
                        "name": "/local_occupancy_grid",
                        "type": "OccupancyGrid",
                    },
                    {
                        "name": "/global_path",
                        "type": "Path",
                        "options": {"color": 0xFF0000},
                    },
                    {
                        "name": "/local_path",
                        "type": "Path",
                        "options": {"color": 0x0000FF},
                    },
                    {
                        "name": "/move_base_simple/goal",
                        "type": "PoseStamped",
                    },
                ],
            },
            {
                "name": "Lidar",
                "type": "PointCloud",
                "description": "Live Lidar data displayed from the robot",
                "topics": [
                    {
                        "name": "/cloud_registered_1",
                        "type": "PointCloud2",
                    }
                ],
            },
            {
                "name": "/cmd_vel",
                "type": "graph",
                "description": "Graph of the command velocity sent to the robot via /cmd_vel",
                "topics": [
                    {
                        "name": "/cmd_vel",
                        "type": "Twist",
                    }
                ],
            },
        ]
        self.sim_objects = [
            {
                "name": "Navigation",
                "type": "3d",
                "description": "Main Navigation display for the G1, contains, current location, live OccupancyGrid, goals, and planned paths. To set a goal via the web interface, hold shift and click on the desired location in the map.",
                "topics": [
                    {
                        "name": "pelvis",
                        "type": "Tf",
                    },
                    {
                        "name": "/map",
                        "type": "OccupancyGrid",
                    },
                    {
                        "name": "/global_path",
                        "type": "Path",
                        "options": {"color": 0xFF0000},
                    },
                    {
                        "name": "/local_path",
                        "type": "Path",
                        "options": {"color": 0x0000FF},
                    },
                    {
                        "name": "/move_base_simple/goal",
                        "type": "PoseStamped",
                    },
                ],
            },
            {
                "name": "Lidar",
                "type": "PointCloud",
                "description": "Live Lidar data displayed from the robot",
                "topics": [
                    {
                        "name": "/map_point_cloud",
                        "type": "PointCloud2",
                    },
                ],
            },
            {
                "name": "/cmd_vel",
                "type": "graph",
                "description": "Graph of the command velocity sent to the robot via /cmd_vel",
                "topics": [
                    {
                        "name": "/cmd_vel",
                        "type": "Twist",
                    }
                ],
            },
            {
                "name": "Rosout",
                "type": "log",
                "description": "Output logs from rosout",
                "topics": [
                    {
                        "name": "/rosout",
                        "type": "rosgraph_msgs/Log",
                    }
                ],
            },
        ]
        self.buttons = {
            "name": "Nav",
            "buttons": [
                {
                    "name": "Start Nav Real",
                    "type": "button",
                    "action": "/nav/start_real",
                    "icon": "assistant_navigation",
                    "color": "green",
                },
                {
                    "name": "Start Nav Sim",
                    "type": "button",
                    "action": "/nav/start_sim",
                    "icon": "map",
                    "color": "green",
                },
            ],
        }

    def register_routes(
        self, app, socketio, send_socket_update, start_process, stop_process
    ):
        @app.route("/nav/start_sim", methods=["POST"])
        def start_nav_sim():
            if start_process(
                self.name,
                self.main_sources,
                self.sim_objects,
                "roslaunch unitree_controller master_sim.launch",
                True,
            ):
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})

        @app.route("/nav/start_real", methods=["POST"])
        def start_nav_real():
            main = start_process(
                self.name,
                self.main_sources,
                self.real_objects,
                "roslaunch unitree_controller master_real.launch",
                False,
            )
            # starts rosbridge to convert cmd_vel to ros2 for connection to robot
            bridge = start_process(
                self.name,
                self.ros_bridge_sources,
                self.real_objects,
                "ros2 launch unitree_link unitree_launch.py",
                False,
            )
            if main and bridge:
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})
