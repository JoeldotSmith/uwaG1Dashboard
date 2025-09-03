from flask import Flask, jsonify, send_from_directory
from flask_socketio import SocketIO
import subprocess
import os
import psutil
import signal
from controllers.navigation.navigation_controller import NavigationController
from controllers.mocopi.mocopi_controller import MocopiController

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


class G1Server:
    def __init__(self):
        self.running_proci = {}
        self.name_of_running_service = "Not Running"
        self.current_topics = []
        self.is_sim = False
        self.controllers = [NavigationController(), MocopiController()]
        self.buttons = []

    def build_source_command(self, cmd, local_sources=None):
        if local_sources:
            sources_cmd = " && ".join(f"source {s}" for s in local_sources)
            return f"bash -c '{sources_cmd} && {cmd}'"
        return f"bash -c '{cmd}'"

    def update_state(self, name, is_sim, topics):
        self.name_of_running_service = name
        self.current_topics = topics
        self.is_sim = is_sim

        send_socket_update(
            "state",
            {
                "running": len(self.running_proci) > 0,
                "name": server.name_of_running_service,
                "topics": server.current_topics,
                "sim": server.is_sim,
            },
        )

    def is_running(self, cmd):
        return cmd in self.running_proci


server = G1Server()


def send_socket_update(header, body):
    global socketio
    socketio.emit(header, body)


def start_process(
    controllerName, controllerSources, controllerObjectsToDisplay, cmd, sim=False
):
    global server
    cmd = server.build_source_command(cmd, controllerSources)
    if not server.is_running(cmd):
        print("Running: " + cmd)
        server.running_proci[cmd] = subprocess.Popen(
            cmd, shell=True, executable="/bin/bash"
        )
        server.update_state(controllerName, sim, controllerObjectsToDisplay)
        return True
    return False


def stop_process():
    global server
    if len(server.running_proci) > 0:
        for _, val in server.running_proci.items():
            val.terminate()

        server.running_proci = {}
        server.update_state("Not Running", False, [])
        return True
    return False


for controller in server.controllers:
    controller.register_routes(
        app, socketio, send_socket_update, start_process, stop_process
    )
    server.buttons.append(controller.buttons)


# Global routes
@app.route("/state", methods=["GET"])
def state():
    send_socket_update(
        "state",
        {
            "running": len(server.running_proci) > 0,
            "name": server.name_of_running_service,
            "sim": server.is_sim,
            "topics": server.current_topics,
        },
    )
    return jsonify({"status": "success"})


@app.route("/buttons")
def get_buttons():
    global server
    return server.buttons


@app.route("/")
def index():
    return app.send_static_file("index.html")


@app.route("/system-stats")
def system_stats():
    stats = {
        "cpu": psutil.cpu_percent(interval=None),
        "memory": psutil.virtual_memory().percent,
        "disk": psutil.disk_usage("/").percent,
    }
    return jsonify(stats)


@app.route("/stop")
def stop_all():
    if stop_process():
        return jsonify({"status": "stopped"})
    return jsonify({"status": "not running"})


def start_base_process():
    rosbridge_cmd = 'bash -c "source /opt/ros/noetic/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch"'
    rosbridge_proc = subprocess.Popen(
        rosbridge_cmd,
        shell=True,
        executable="/bin/bash",
    )

    tf_repub_cmd = 'bash -c "source /opt/ros/noetic/setup.bash && rosrun tf2_web_republisher tf2_web_republisher"'
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

def start():
    import subprocess
    rosbridge_proc, tf_repub_proc = start_base_process()
    try:
        import gunicorn.app.wsgiapp as wsgi
        sys.argv = ['gunicorn', '--bind', '0.0.0.0:8000', '--worker-class', 'eventlet', '-w', '1', 'dashboard_server:app']
        wsgi.run()
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_processes([rosbridge_proc, tf_repub_proc] + list(server.running_proci.values()))

if __name__ != "__main__":
    start_base_process()
