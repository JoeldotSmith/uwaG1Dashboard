import subprocess, os, signal

class BaseController:
    def __init__(self):
        self.local_sources = []
        self.objects_to_display = []
        self.name = self.__class__.__name__

    def register_routes(self, app, socketio, send_socket_update, start_process, stop_process):
        raise NotImplementedError
