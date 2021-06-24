import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from "../console/Console.js";
import WidgetTreeDebugger from "../../tools/WidgetTreeDebugger";

import "./GUI.css";

import WT from "../../datastructs/WidgetTree.js";

class GUI extends React.Component {
  state = {
    websocket: null,
    settings: {
      keybindings: {}
    },
    windows: new WT.Window(),
    consoleShow: true,
  };

  constructor(props) {
    super(props);

    this.state.websocket = require("socket.io-client")("http://localhost:4040");
    let window2 = new WT.Window();
    window2 = WT.add(window2, new WT.Leaf("key_controller"));
    window2 = WT.add(window2, new WT.Leaf("settings"));
    let window3 = new WT.Window();
    window3 = WT.add(window3, new WT.Leaf("ros_camera"));
    window3 = WT.add(window3, new WT.Leaf("controller"));
    window3 = WT.add(window3, new WT.Leaf("ros_camera"));
    // window3 = WT.add(window3, new WT.Leaf("controller"));
    window2 = WT.add(window2, window3);
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("ros_camera"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("ip_camera"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("ip_camera"));
    this.state.windows = WT.add(this.state.windows, window2);
    WT.averageSize(
      this.state.windows,
      window.innerWidth,
      window.innerHeight - 80
    );
    window.addEventListener("resize", () =>
      WT.handleResize(this.state.windows, this.updateWidgets)
    );
  }

  addWidget = (widgetName) => {
    let root = this.state.windows;
    switch (widgetName) {
      case "Settings":
        WT.add(root, new WT.Leaf("settings"));
        this.setState({
          windows: root,
        });
        break;
      case "IP Camera":
        WT.add(root, new WT.Leaf("ip_camera"));
        this.setState({
          windows: root,
        });
        break;
      case "ROS Camera":
        WT.add(root, new WT.Leaf("ros_camera"));
        this.setState({
          windows: root,
        });
        break;
      case "Controller":
        WT.add(root, new WT.Leaf("controller"));
        this.setState({
          windows: root,
        });
        break;
      case "Console":
        this.setState({
          consoleShow: !this.state.consoleShow,
        });
        break;
      case "Script Runner":
        WT.add(root, new WT.Leaf("script_runner"));
        this.setState({
          windows: root
        });
        break;
    }
  };

  //Render Nav Bar, Widget Display, Console, and Settings
  render() {
    return (
      <div className="gui">
        <Navbar addWidget={this.addWidget} removeWidget={this.removeWidget} />
        <Console show={this.state.consoleShow} />

        <div className="widgetDisplay">
          {
            WT.renderWindows(this.state.windows, this.updateWidgets)
          }
          <WidgetTreeDebugger tree={this.state.windows}/>

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
