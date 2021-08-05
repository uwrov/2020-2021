import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from "../console/Console.js";
import WidgetWindow from "../widgetwindow/WidgetWindow.js";
import {Widget} from "../widgets/Widget.js";

import "./GUI.css";

class GUI extends React.Component {
  state = {
    websocket: null,
    settings: {
      keybindings: {}
    },
    consoleShow: false,
    widgets: [new Widget("ros_camera"), new Widget("controller"), new Widget("script_runner")]
  };


  addWidget = (widgetName) => {
    let widgets = this.state.widgets;
    switch (widgetName) {
      case "Settings":
        widgets.push(new Widget("settings"));
        this.setState({
          widgets: widgets,
        });
        break;
      case "IP Camera":
        widgets.push(new Widget("ip_camera"));
        this.setState({
          widgets: widgets,
        });
        break;
      case "ROS Camera":
        widgets.push(new Widget("ros_camera"));
        this.setState({
          widgets: widgets,
        });
        break;
      case "Controller":
        widgets.push(new Widget("controller"));
        this.setState({
          widgets: widgets,
        });
        break;
      case "Console":
        this.setState({
          consoleShow: !this.state.consoleShow,
        });
        break;
      case "Script Runner":
        widgets.push(new Widget("script_runner"));
        this.setState({
          widgets: widgets,
        });
        break;
    }
  };

  //Render Nav Bar, Widget Display, Console, and Settings
  render() {
    return (
      <div className="gui">
        <Navbar addWidget={this.addWidget} />
        <Console show={this.state.consoleShow} />
        <div className="widgetDisplay">
          <WidgetWindow widgets={this.state.widgets} update={this.updateWidgets}/>
        </div>
      </div>
    );
  }

  updateWidgets = (newWidgets) => {
    this.setState({ widgets: newWidgets });
  };

  updateSettings = (newSettings) => {
    this.setState({ settings: newSettings });
  };
}

export default GUI;
