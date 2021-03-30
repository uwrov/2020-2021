import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from "../console/Console.js";
import WidgetTreeDebugger from "../../tools/WidgetTreeDebugger";

import "../widgets/Widget.css";
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
    window2 = WT.add(window2, new WT.Leaf("widget"));
    window2 = WT.add(window2, new WT.Leaf("widget"));
    let window3 = new WT.Window();
    window3 = WT.add(window3, new WT.Leaf("widget"));
    window3 = WT.add(window3, new WT.Leaf("widget"));
    window2 = WT.add(window2, window3);
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("widget"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("widget"));
    this.state.windows = WT.add(this.state.windows, new WT.Leaf("widget"));
    this.state.windows = WT.add(this.state.windows, window2);
  }

  addWidget = (widgetName) => {
    console.log(widgetName);
    this.addTab(widgetName, 0);
    switch (widgetName) {
      case "settings":
        //this.addTab("settings", 1);
        break;
      case "mainCam":
        // new Camera();
        break;
      case "controller":
        //new Controller();
        break;
      case "console":
        //new Console();
        break;
    }
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
