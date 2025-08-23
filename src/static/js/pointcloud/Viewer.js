"use strict";

class Viewer {
  /**
    * Class constructor.
    * @constructor
  **/
  constructor(card, topicName, topicType) {
    this.card = card;

    this.topicName = topicName;
    this.topicType = topicType;

    let that = this;
    let menuId = 'menu-' + Math.floor(Math.random() * 1e6);
    let viewers = Viewer.getViewersForType(this.topicType);
    for(let i in viewers) {
      let item = $('<li ' + (viewers[i].name === this.constructor.name ? 'disabled' : '') + ' class="mdl-menu__item">' + viewers[i].friendlyName + '</li>').appendTo(this.card.menu);
      let that = this;
      item.click(() => { Viewer.onSwitchViewer(that, viewers[i]); });
    }
    this.lastDataTime = 0.0;
    this.onCreate();
  }

  destroy() {
    this.card.empty();
  }

  onResize() { }

  onDataPaused(data) { }

  onData(data) { }

  update(data) {
    let time = Date.now();
    if( (time - this.lastDataTime)/1000.0 < 1/this.constructor.maxUpdateRate - 5e-4) {
      return;
    }

    if(this.isPaused) { this.onDataPaused(data); return; }

    this.lastDataTime = time;

    // get rid of the spinner
    if(this.loaderContainer) {
      this.loaderContainer.remove();
      this.loaderContainer = null;
    }

    if(data._error) {
      this.error(data._error);
      return;
    }

    if(data._warn) {
      this.warn(data._warn);
    }

    // actually update the data
    this.onData(data);
  }
}

Viewer.friendlyName = "Viewer";

// can be overridden by child class
// list of supported message types by viewer, or "*" for all types
// todo: support regexes?
Viewer.supportedTypes = [];

// can be overridden by child class
// max update rate that this viewer can handle
// for some viewers that do extensive DOM manipulations, this should be set conservatively
Viewer.maxUpdateRate = 50.0;

// not to be overwritten by child class!
// stores registered viewers in sequence of loading
Viewer._viewers = [];

// override this
Viewer.onClose = (viewerInstance) => { console.log("not implemented; override necessary"); }
Viewer.onSwitchViewer = (viewerInstance, newViewerType) => { console.log("not implemented; override necessary"); }

// not to be overwritten by child class!
Viewer.registerViewer = (viewer) => {
  // registers a viewer. the viewer child class calls this at the end of the file to register itself
  Viewer._viewers.push(viewer);
};

// not to be overwritten by child class!
Viewer.getDefaultViewerForType = (type) => {
  // gets the viewer class for a given message type (e.g. "std_msgs/msg/String")

  // if type is "package/MessageType", converted it to "package/msgs/MessageType"
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }

  // go down the list of registered viewers and return the first match
  for(let i in Viewer._viewers) {
    if(Viewer._viewers[i].supportedTypes.includes(type)) {
      return Viewer._viewers[i];
    }
    if(Viewer._viewers[i].supportedTypes.includes("*")) {
      return Viewer._viewers[i];
    }
  }
  return null;
}

// not to be overwritten by child class!
Viewer.getViewersForType = (type) => {
  // gets the viewer classes for a given message type (e.g. "std_msgs/msg/String")

  let matchingViewers = [];

  // if type is "package/MessageType", converted it to "package/msgs/MessageType"
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }

  // go down the list of registered viewers and return the first match
  for(let i in Viewer._viewers) {
    if(Viewer._viewers[i].supportedTypes.includes(type)) {
      matchingViewers.push(Viewer._viewers[i]);
    }
    if(Viewer._viewers[i].supportedTypes.includes("*")) {
      matchingViewers.push(Viewer._viewers[i]);
    }
  }

  return matchingViewers;
}
