import React from "react";
import ReactDOM from "react-dom";
import "./index.css";
import GUI from "./components/gui/GUI.js";
import RosCamera from "./components/rosCamera/RosCamera.js";
import IpCamera from "./components/ipCamera/IpCamera.js";
import Xbox from "./components/xbox/Xbox.js";
import Controller from "./components/controller/Controller.js";

import Tests from "./test-environment/WidgetTreeTest/WidgetTreeTest.js";

ReactDOM.render(<GUI />, document.getElementById('root'));

// ReactDOM.render(<Console />, document.getElementById('root'));

//ReactDOM.render(<Xbox />, document.getElementById('root'));

//let button = new Button();
//ReactDOM.render(<GUI />, document.getElementById("root"));
// ReactDOM.render(<Controller />, document.getElementById('root'));
//ReactDOM.render(button.render(), document.getElementById('root'));
// ReactDOM.render(<IpCamera />, document.getElementById('root'));
ReactDOM.render(<RosCamera />, document.getElementById('root'));
//ReactDOM.render(<IpCamera />, document.getElementById('root'));

//Tests.runAllTests();
