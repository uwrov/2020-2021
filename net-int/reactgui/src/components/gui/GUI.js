import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
import Settings from "../Settings/Settings.js";
//import Controller from "../controller/Controller.js";
import Camera from "../camera/Camera.js";
import WidgetDisplay from "../widgets/WidgetDisplay.js";
import "./GUI.css";

class GUI extends React.Component {
   state = {
      websocket: null,
      settings: {},
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');
   }

   addWidget = (widgetName) => {
      	console.log(widgetName);
       	switch (widgetName) {
	       case "settings":
	          new Settings();
	          break;
	       case "mainCam":
	          new Camera();
	          break;
	       case "controller":
	          //new Controller();
	          break;
	       case "console":
	          new Console();
	          break;
	    }
   }
	//Render Nav Bar, Widget Display, Console, and Settings
   render() {
      return (
         <div>
        	<Navbar addWidget = {this.addWidget}/>
        	<Console />   
         </div>
      );
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }

}

export default GUI;
