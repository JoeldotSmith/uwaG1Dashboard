from flask import jsonify
import os
from controllers.base_controller import BaseController


class MocopiController(BaseController):
    def __init__(self):
        self.name = "Mocopi"
        self.real_sources = [
            "/opt/ros/foxy/setup.bash",
            os.path.expanduser("~/unitree_ros2_default/setup.sh"),
        ]
        self.test_sources = [
            "/opt/ros/foxy/setup.bash",
            os.path.expanduser("~/unitree_ros2_default/setup-local.sh"),
        ]
        self.real_objects = []
        self.sim_objects = []
        self.buttons = {
            "name": "Mocopi",
            "buttons": [
                {
                    "name": "Start Mocopi Real",
                    "type": "button",
                    "action": "/mocopi/start_real",
                    "icon": "connect_without_contact",
                    "color": "green",
                },
                {
                    "name": "Start Mocopi Sim",
                    "type": "button",
                    "action": "/mocopi/start_sim",
                    "icon": "connected_tv",
                    "color": "green",
                },
            ],
        }

    def register_routes(
        self, app, socketio, send_socket_update, start_process, stop_process
    ):
        @app.route("/mocopi/start_sim", methods=["POST"])
        def start_mocopi_real_sim():
            # run log data
            log = start_process(
                self.name,
                [],
                self.sim_objects,
                "python3 home/joelsmith/unitree_ros2_default/unitree_ws/src/mocopi_ros2/pass_log_data.py",
                True,
            )

            # run display
            display = start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "ros2 launch mocopi_ros2 display.launch.py",
                True,
            )

            # run collision checker
            collision = start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "ros2 run fcl_self_collision_checker collision_checker",
                True,
            )

            if log and display and collision:
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})

        @app.route("/mocopi/start_real", methods=["POST"])
        def start_mocopi_real():
            # run display
            display = start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "ros2 launch mocopi_ros2 display.launch.py",
                True,
            )

            # run collision checker
            collision = start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "ros2 run fcl_self_collision_checker collision_checker",
                True,
            )

            # run pose_follower_node
            pose = start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "ros2 run g1_pose_follower pose_follower_node",
                True,
            )

            if pose and display and collision:
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})
