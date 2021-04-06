import React from "react";

import Console from "../components/console/Console.js";
import Settings from "../components/settings/Settings.js";
import Controller from "../components/controller/Controller.js";
import RosCamera from "../components/rosCamera/RosCamera.js";
import IpCamera from "../components/ipCamera/IpCamera.js";

import TestWidget from "../components/widgets/TestWidget.js";
import Xbox from "../components/xbox/Xbox.js";

export let WIDGET_DICT = {
  settings: <Settings />,
  ros_camera: <RosCamera />,
  ip_camera: <IpCamera />,
  widget: <TestWidget />,
  console: <Console />,
  controller: <Xbox />,
  key_controller: <Controller />
};

export default null;
