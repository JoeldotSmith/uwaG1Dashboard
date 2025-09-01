class ROSDisplayManager {
  constructor(containerId, ros) {
    this.container = document.getElementById(containerId);
    this.ros = ros;
    this.tfClient = new ROSLIB.TFClient({
      ros: ros,
      fixedFrame: "map",
      angularThres: 0.01,
      transThres: 0.01,
      serverName: "/tf2_web_republisher"
    });

    // {id: "id of wrapper" subscriptions: [subscription objects]}
    this.group_subscriptions = [];

    // bad fix, for inverted colour bug
    ROS3D.OccupancyGrid.prototype.getColor = function(index, row, col, value) {
      return [
        255 - ((value * this.color.r) / 255),
        255 - ((value * this.color.g) / 255),
        255 - ((value * this.color.b) / 255),
        255
      ];
    };
  }

  clearDisplays() {
    this.container.innerHTML = "";
    // this.group_subscriptions.forEach(group => {
    //   group.subscriptions.forEach(sub => {
    //     try {
    //       sub.unsubscribe());
    //     } catch (error) {
    //       console.warn(error);
    //     }
    //   });
    // });
    // this.group_subscriptions = [];
  }

  createDisplays(topicgroups) {
    this.clearDisplays();
    topicgroups.forEach(group => {
      this.createDisplay(group);
    })
  }

  createDisplay(group) {
    switch (group.type) {
      case "3d":
        this._handle3d(group)
        break;
      case "PointCloud":
        this._handlePointCloud(group)
        break;
      case "graph":
        this._handleGraph(group)
        break;
      default:
        console.warn("Unsupported topic type:", group.type);
        return null;
    }
  }

  _createDisplayWrapper(title) {
    const wrapper = document.createElement("div")
    wrapper.className = "flex flex-col p-2 secondary rounded border border-coloured"
    wrapper.style.maxHeight = "100%";
    const heading = document.createElement("h2")
    heading.textContent = `${title}`
    heading.className = "text-lg font-bold p-2 text-white"
    wrapper.appendChild(heading)
    const div = document.createElement("div");
    div.className = "w-full border-coloured h-0";
    wrapper.appendChild(div);
    return wrapper
  }

  _handlePointCloud(group) {
    const wrapper = this._createDisplayWrapper(group.name);
    const div = document.createElement("div");
    div.className = "w-full flex h-full pt-4 setWhite rounded";
    wrapper.appendChild(div);
    this.container.appendChild(wrapper);

    const uniqueId = `viewer-3d-${Date.now()}-${Math.floor(Math.random() * 1000)}`;
    div.id = uniqueId;

    group.topics.forEach(topic => {
      const sub = new ROSLIB.Topic({
        ros: this.ros,
        name: topic.name,
        messageType: "sensor_msgs/PointCloud2",
        throttle_rate: 10,
      });
      const viewer = new PointCloud2Viewer(div, topic.name, "sensor_msgs/msg/PointCloud2");

      sub.subscribe((msg) => {
        viewer.onData(msg);
      });
    });
  }


  // MAIN HANDLERS
  _handle3d(group) {
    const wrapper = this._createDisplayWrapper(group.name);
    const div = document.createElement("div");
    div.className = "w-full flex h-full pt-4 setWhite rounded";
    wrapper.appendChild(div);
    this.container.appendChild(wrapper);

    const uniqueId = `viewer-3d-${Date.now()}-${Math.floor(Math.random() * 1000)}`;
    div.id = uniqueId;

    const viewer = new ROS3D.Viewer({
      divID: uniqueId,
      width: div.clientWidth,
      height: div.clientHeight,
      antialias: true
    });

    group.topics.forEach(topic => {
      switch (topic.type) {
        case "OccupancyGrid":
          this._createOccupancyGrid(topic, viewer);
          break;
        case "Tf":
          this._createTf(topic, viewer);
          break;
        case "PoseStamped":
          this._createPoseStamped(topic, viewer);
          break;
        case "Path":
          this._createPath(topic, viewer);
          break;
        case "URDF":
          this._createURDF(topic, viewer);
          break;
        default:
          console.warn("Unsupported topic type:", topic.type);
          return null;
      }
    })
    const resizeObserver = new ResizeObserver(entries => {
      for (let entry of entries) {
        const { width, height } = entry.contentRect;
        viewer.resize(width, height);
      }
    });
    resizeObserver.observe(div);
  }

  _handleGraph(group) {
    const wrapper = this._createDisplayWrapper(group.name);
    const div = document.createElement("div");
    div.className = "w-full h-full pt-4 setWhite rounded flex flex-col";
    div.style = "position: relative";
    wrapper.appendChild(div);
    this.container.appendChild(wrapper);

    const canvas = document.createElement("canvas");
    canvas.style.width = "100%";
    canvas.style.height = "100%";
    div.appendChild(canvas);

    const ctx = canvas.getContext("2d");

    const data = {
      labels: [],
      datasets: [
        { label: "Linear (x)", data: [], borderColor: "red", fill: true },
        { label: "Angular (y)", data: [], borderColor: "blue", fill: true },
      ]
    };
    const chart = new Chart(ctx, {
      type: "line",
      data: data,
      options: {
        animation: false,
        responsive: true,
        scales: {
          x: { display: true, title: { display: true, text: "Time" } },
          y: { display: true, title: { display: true, text: "Value" } },
        }
      }
    });

    group.topics.forEach(topic => {
      switch (topic.type) {
        case "Twist":
          this._createTwist(topic, chart, data);
          break;
        default:
          console.warn("Unsupported topic type:", topic.type);
          return null;

      }
    })
    const resizeObserver = new ResizeObserver(() => chart.resize());
    resizeObserver.observe(div);
  }

  // 3D SPECIFIC HANDLERS
  _createTf(topic, viewer) {
    const robotMarker = new ROS3D.Arrow({
      shaftDiameter: 0.05,
      headDiameter: 0.1,
      headLength: 0.2,
      material: new THREE.MeshBasicMaterial({ color: 0xff0000 })
    });
    viewer.scene.add(robotMarker);

    this.tfClient.subscribe(topic.name, tf => {
      robotMarker.position.set(tf.translation.x, tf.translation.y, tf.translation.z);
      const rot90 = new THREE.Quaternion();
      rot90.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

      const q = tf.rotation;
      const tfQuat = new THREE.Quaternion(q.x, q.y, q.z, q.w);
      tfQuat.multiply(rot90);
      robotMarker.quaternion.copy(tfQuat);
    });
  }

  _createOccupancyGrid(topic, viewer) {
    const gridClient = new ROS3D.OccupancyGridClient({
      ros: this.ros,
      rootObject: viewer.scene,
      continuous: true,
      topic: topic.name,
    });
  }

  _createPath(topic, viewer) {
    const defaults = {
      ros: this.ros,
      rootObject: viewer.scene,
      tfClient: this.tfClient,
      topic: topic.name,
      color: 0x00ffff
    };

    const pathOptions = topic.options ? { ...defaults, ...topic.options } : defaults;
    new ROS3D.Path(pathOptions);
  }

  _createURDF(topic, viewer) {
    // cant get this to work !!!!!

    console.warn("URDF not working yet!")

    //
    // topic.name = "robot_description"
    // viewer = ROS3D.Viewer
    // const defaults = {
    //   ros: this.ros,
    //   tfClient: this.tfClient,
    //   path : `http://${window.location.hostname}:8000/urdf/`,
    //   loader: ROS3D.STL_LOADER,
    //   rootObject: viewer.scene,
    //   param: topic.name,
    // };
    //
    // const options = topic.options ? { ...defaults, ...topic.options } : defaults;
    // setTimeout(() => {new ROS3D.UrdfClient(options);}, 5000);
    // new ROS3D.UrdfClient({
    //   ros: this.ros,
    //   tfClient: this.tfClient,
    //   path : `http://${window.location.hostname}:8000/urdf/`,
    //   rootObject: viewer.scene,
    // });
  }

  _createPoseStamped(topic, viewer) {
    const poseMarker = new ROS3D.Arrow({
      shaftDiameter: 0.05,
      headDiameter: 0.1,
      headLength: 0.2,
      material: new THREE.MeshBasicMaterial({ color: 0xff00ff }),
    });
    viewer.scene.add(poseMarker);

    const poseTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: topic.name,
      messageType: "geometry_msgs/PoseStamped",
    });

    poseTopic.subscribe(msg => {
      const p = msg.pose.position;
      const o = msg.pose.orientation;

      poseMarker.position.set(p.x, p.y, p.z);
      const rot90 = new THREE.Quaternion();
      rot90.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

      const tfQuat = new THREE.Quaternion(o.x, o.y, o.z, o.w);
      tfQuat.multiply(rot90);
      poseMarker.quaternion.copy(tfQuat);
    });
  }

  // GRAPH HANDLERS
  _createTwist(topic, chart, data) {
    const twistTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: topic.name,
      messageType: "geometry_msgs/Twist"
    });

    twistTopic.subscribe(msg => {
      const timestamp = new Date().toLocaleTimeString();
      data.labels.push(timestamp);
      if (data.labels.length > 50) data.labels.shift();

      data.datasets[0].data.push(msg.linear.x);
      data.datasets[1].data.push(msg.angular.z);

      for (let ds of data.datasets) {
        if (ds.data.length > 50) ds.data.shift();
      }
      chart.update("none");
    });
  }

}
