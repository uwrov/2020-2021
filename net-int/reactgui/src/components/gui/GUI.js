import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from "../console/Console.js";
import WidgetTreeDebugger from "../../tools/WidgetTreeDebugger";

import "./GUI.css";

import WT from "../../datastructs/WidgetTree.js";

class GUI extends React.Component {
  state = {
    websocket: null,
    settings: {},
    windows: new WT.Window(),
  };

  constructor(props) {
    super(props);

    this.state.websocket = require("socket.io-client")("http://localhost:4040");
    let window2 = new WT.Window();
    window2 = WT.add(window2, new WT.Leaf("console"));
    window2 = WT.add(window2, new WT.Leaf("widget"));
    let window3 = new WT.Window();
    window3 = WT.add(window3, new WT.Leaf("ros_camera "));
    window3 = WT.add(window3, new WT.Leaf("controller"));
    window2 = WT.add(window2, window3);
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("widget"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("ip_camera"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("settings"));
    this.state.windows = WT.add(this.state.windows, window2);
    WT.averageSize(this.state.windows, window.innerWidth, 700);
  }

  addWidget = (widgetName) => {
    console.log(widgetName);
    let root = this.state.windows;
    switch (widgetName) {
      case "settings":
        WT.add(root, new WT.Leaf("settings"));
        break;
      case "mainCam":
        WT.add(root, new WT.Leaf("ip_camera"));
        break;
      case "controller":
        WT.add(root, new WT.Leaf("controller"));
        break;
      case "console":
        WT.add(root, new WT.Leaf("console"));
        break;
    }
    this.setState({
      windows: root
    });
  };

  addWidget = (widgetName) => {
    console.log(widgetName);
    let root = this.state.windows;
    switch (widgetName) {
      case "settings":
        WT.add(root, new WT.Leaf("settings"));
        break;
      case "mainCam":
        WT.add(root, new WT.Leaf("ip_camera"));
        break;
      case "controller":
        WT.add(root, new WT.Leaf("controller"));
        break;
      case "console":
        WT.add(root, new WT.Leaf("console"));
        break;
    }
    this.setState({
      windows: root
    });
  };

  //Render Nav Bar, Widget Display, Console, and Settings
  render() {
    return (
      <div className="gui">
        <Navbar addWidget={this.addWidget} removeWidget={this.removeWidget} />
        <Console />

        <div className="widgetDisplay">
          {
            WT.renderWindows(this.state.windows, this.updateWidgets)
            //Render Widgets
            //          <WidgetTreeDebugger tree={this.state.windows}/>
          }
        </div>
      </div>
    );
  }

  updateWidgets = (newWidgets) => {
    this.setState({ windows: newWidgets });
  };

  updateSettings = (newSettings) => {
    this.setState({ settings: newSettings });
  };
}

export default GUI;
