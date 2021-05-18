import React from "react";

import Console from "../components/console/Console.js";
import Settings from "../components/settings/Settings.js";
import Controller from "../components/controller/Controller.js";
import RosCamera from "../components/rosCamera/RosCamera.js";
import IpCamera from "../components/ipCamera/IpCamera.js";

import TestWidget from "../components/widgets/TestWidget.js";
import Xbox from "../components/xbox/Xbox.js";

export let WIDGET_DICT = (data) => {
  switch(data.type) {
    case "settings":
      return <Settings props={data.savedProps}/>
    case "ros_camera":
      return <RosCamera props={data.savedProps}/>
    case "ip_camera":
      return <IpCamera props={data.savedProps}/>
    case "controller":
      return <Xbox props={data.savedProps}/>
    case "key_controller":
      return <Controller props={data.savedProps}/>
  }
}

export default null;
