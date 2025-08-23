from flask import Flask, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import subprocess
import threading
import os
import signal
from controllers.navigation.navigation_controller import NavigationController

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


class G1Server:
    def __init__(self):
        self.running_process = None
        self.name_of_running_service = "Not Running"
        self.current_topics = []
        self.is_sim = False
        self.controllers = [
            NavigationController(),
        ]

        self.global_sources = [
            "/opt/ros/noetic/setup.bash",
        ]

    def build_source_command(self, cmd, local_sources=None):
        all_sources = self.global_sources[:]  
        if local_sources:
            all_sources += local_sources
        sources_cmd = " && ".join(f"source {s}" for s in all_sources)
        return f"bash -c '{sources_cmd} && {cmd}'"

    def update_state(self, name, is_sim, topics):
        self.name_of_running_service = name
        self.current_topics = topics
        self.is_sim = is_sim

        send_socket_update("state", {
            "running": server.is_running(),
            "name": server.name_of_running_service,
            "topics": server.current_topics,
            "sim": server.is_sim
        })

    def is_running(self):
        return self.running_process is not None

server = G1Server()


def send_socket_update(header, body):
    global socketio
    socketio.emit(header, body)

def start_process(controller, cmd, sim=False):
    global server
    cmd = server.build_source_command(cmd, controller.local_sources)
    if not server.is_running():
        server.running_process = subprocess.Popen(cmd, shell=True, executable="/bin/bash")
        server.update_state(controller.name, sim, controller.objects_to_display)
        return True
    return False

def stop_process():
    global server
    if server.is_running():
        server.running_process.terminate()
        server.running_process = None
        server.update_state("Not Running", False, [])
        return True
    return False


for controller in server.controllers:
    controller.register_routes(app, socketio, send_socket_update, start_process, stop_process)


# Global routes
@app.route("/state", methods=["GET"])
def state():
    send_socket_update("state", {
        "running": server.is_running(),
        "name": server.name_of_running_service,
        "sim": server.is_sim,
        "topics": server.current_topics
    })
    return jsonify({"status": "success"})

@app.route("/")
def index():
    return app.send_static_file("index.html")

def start_base_process():
    rosbridge_cmd = server.build_source_command("roslaunch rosbridge_server rosbridge_websocket.launch")
    rosbridge_proc = subprocess.Popen(
        rosbridge_cmd,
        shell=True,
        executable="/bin/bash",
    )

    tf_repub_cmd = server.build_source_command("rosrun tf2_web_republisher tf2_web_republisher")
    tf_repub_proc = subprocess.Popen(
        tf_repub_cmd,
        shell=True,
        executable="/bin/bash",
    )

    return rosbridge_proc, tf_repub_proc

def cleanup_processes(procs):
    """Gracefully terminate ROS processes on exit."""
    for proc in procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

# Serve URDF files for rendering
@app.route("/urdf/<path:filename>")
def serve_urdf(filename):
    return send_from_directory(os.path.join(app.root_path, "urdf"), filename)

if __name__ == "__main__":
    import os
    rosbridge_proc, tf_repub_proc = start_base_process()

    try:
        socketio.run(app, host="0.0.0.0", port=8000)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_processes([rosbridge_proc, tf_repub_proc, server.running_process])

