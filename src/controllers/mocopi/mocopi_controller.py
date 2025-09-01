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
            if start_process(
                self.name,
                self.test_sources,
                self.sim_objects,
                "roslaunch unitree_controller master_sim.launch",
                True,
            ):
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})

        @app.route("/mocopi/start_real", methods=["POST"])
        def start_mocopi_real():
            if start_process(
                self.name,
                self.real_sources,
                self.real_objects,
                "roslaunch unitree_controller master_real.launch",
                False,
            ):
                return jsonify({"status": "started"})
            return jsonify({"status": "already running"})
