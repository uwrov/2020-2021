import React from "react";
import Settings from "./Settings";
import MainCam from "./MainCam";
import NavBar from "./NavBar";
import Widgets from "./Widgets";
import MiniCam from "./MiniCam";

import Console from "../../components/console/Console";
import Controller from "../../components/controller/Controller";

class GUI extends React.Component {
  state = {
    cam_ip: "localhost",
    cam_ports: ["8080", "8081", "8082", "8083", "8084"],
    main_cam_index: 0,
    shownComponents: [],
    ip_only: false,
  };

  buttons = [
    {
      text: "Settings",
      onClick: () => {
        this.showComponent("settings");
      },
    },
    {
      text: "Debugger",
      onClick: () => {
        this.showComponent("debugger");
      },
    },
  ];

  constructor(props) {
    super(props);

    let ip = window.localStorage.getItem("cam_ip");
    if (ip !== null) {
      this.state.cam_ip = ip;
    }
    let ports = window.localStorage.getItem("ports");
    if (ports !== null) {
      this.state.cam_ports = JSON.parse(ports);
    }
  }
  render() {
    return (
      <div>
        <NavBar buttons={this.buttons} />
        <MainCam
          ip={
            !this.state.ip_only
              ? this.state.cam_ip +
                ":" +
                this.state.cam_ports[this.state.main_cam_index]
              : this.state.cam_ports[this.state.main_cam_index]
          }
        />
        {this.renderSettings()}
        <Widgets
          ip={this.state.cam_ip}
          camPorts={this.state.cam_ports}
          mainIndex={this.state.main_cam_index}
        />
        <Console />
        <Controller />
      </div>
    );
  }

  renderSettings() {
    if (this.state.shownComponents.indexOf("settings") !== -1)
      return (
        <Settings
          onSave={this.handleSettings}
          onExit={() => this.removeComponent("settings")}
        />
      );
  }

  handleSettings = (state) => {
    this.setState({
      cam_ip: state.ip,
      cam_ports: state.ports.map((port) => port.value),
      ip_only: state.ip_only,
    });
  };

  showComponent(name) {
    let visible = this.state.shownComponents.slice(0);
    if (visible.indexOf(name) === -1) visible.push(name);
    this.setState({ shownComponents: visible });
  }

  removeComponent(name) {
    let visible = this.state.shownComponents.slice(0);
    let index = visible.indexOf(name);
    if (index !== -1) visible.splice(index, 1);
    this.setState({ shownComponents: visible });
  }

  getMainIp() {}
}

export default GUI;
