# UWA G1 Dashboard

A lightweight web-based dashboard for monitoring and controlling the UWA G1 humanoid robot.  
It provides visualization of ROS2 data (maps, TFs, lidar point clouds, paths, velocity graphs, etc.) and system resource usage (CPU, RAM, disk, temperature).

## Features
- 2D map visualization with `ros2djs`
- 3D visualization (TFs, PointCloud2, Paths, OccupancyGrids) with `ros3djs` + `three.js`
- Live ROS topic graphs via `Chart.js`
- Sidebar with configurable and reorderable panels
- System stats (CPU, memory, disk, temperature)
- Flask backend serving frontend and stats API
- Single-command deployment, portable setup

## Requirements
### ROS side
- `rosbridge_server` running (for WebSocket communication)
- `tf2_web_republisher` (for TFs)

### Web side
- Python 3.8+

Install all Python dependencies:
```bash
cd uwaG1Dashboard
python3 -m venv env # create virtual environment
source env/bin/activate
pip install -r requirements.txt
```
## Running
``` bash
cd uwaG1Dashboard/src # important to be in the src directory to use the run.sh script else you can run manually
./src/run.sh
```
Default web deployment on  connection https://<host>:8000
