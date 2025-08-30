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
            os.path.expanduser("~/nav_stack/ws_nav/devel/setup.bash")
        ]
        self.ros_bridge_sources = [
            "/opt/ros/noetic/setup.bash",
            "/opt/ros/foxy/setup.bash",
            os.path.expanduser("~/nav_stack/unitree_controller_ros2/setup.sh")
        ]

        self.real_objects = [
            {
                "name": "Navigation",
                "type": "3d",
                "topics": [
                    {
                        "name": "pelvis",
                        "type": "tf",
                    },
                    {
                        "name": "/local_occupancy_grid",
                        "type": "OccupancyGrid",
                    },
                    {
                        "name": "/global_path",
                        "type": "Path",
                        "options": {"color": 0xff0000},
                    },
                    {
                        "name": "/local_path",
                        "type": "Path",
                        "options": {"color": 0x0000ff},
                    },
                    {
                        "name": "/move_base_simple/goal",
                        "type": "PoseStamped",
                    },
                ]
            },
            {
                "name": "Lidar",
                "type": "3d",
                "topics": [
                    {
                        "name": "/cloud_registered_1",
                        "type": "PointCloud2",
                    }
                ]
            },
            {
                "name": "/cmd_vel",
                "type": "graph",
                "topics": [
                    {
                        "name": "/cmd_vel",
                        "type": "Twist",
                    }
                ]
            },
        ]
        self.sim_objects = [
            {
                "name": "Navigation",
                "type": "3d",
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
                        "options": {"color": 0xff0000},
                    },
                    {
                        "name": "/local_path",
                        "type": "Path",
                        "options": {"color": 0x0000ff},
                    },
                    {
                        "name": "/move_base_simple/goal",
                        "type": "PoseStamped",
                    },
                ]
            },
            # {
            #     "name": "Lidar",
            #     "type": "PointCloud",
            #     "topics": [
            #         {
            #             "name": "/map_point_cloud",
            #             "type": "PointCloud2",
            #
            #         },
            #     ],
            #
            # },
            {
                "name": "/cmd_vel",
                "type": "graph",
                "topics": [
                    {
                        "name": "/cmd_vel",
                        "type": "Twist",
                    }
                ],

            }
        ]

    def register_routes(self, app, socketio, send_socket_update, start_process, stop_process):
        @app.route("/nav/start_sim", methods=["POST"])
        def start_nav_sim():
            if start_process(self.name, self.main_sources, self.sim_objects, "roslaunch unitree_controller master_sim.launch", True):
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})

        @app.route("/nav/start_real", methods=["POST"])
        def start_nav_real():
            main = start_process(self.name, self.main_sources, self.real_objects, "roslaunch unitree_controller master_real.launch", False)
            # starts rosbridge to convert cmd_vel to ros2 for connection to robot
            bridge = start_process(self.name, self.ros_bridge_sources, self.sim_objects, "ros2 launch unitree_link unitree_launch.py", True)
            if main and bridge:
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})

        @app.route("/nav/stop", methods=["POST"])
        def stop_nav():
            if stop_process():
                return jsonify({"status": "stopped"})
            return jsonify({"status": "not running"})
