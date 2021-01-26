import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
import Settings from "../Settings/Settings.js";
//import Controller from "../controller/Controller.js";
import Camera from "../camera/Camera.js";
import WidgetDisplay from "../widgets/WidgetDisplay.js";
import Widget from "../widgets/Widget.js";

import "./GUI.css";

class GUI extends React.Component {
   state = {
      websocket: null,
      settings: {},
      windows: []
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');

      this.createWindow(new Widget());
      this.addTab(new Widget("Widget 2"), 0);
      this.addTab(new Widget("Widget 3"), 0);
      this.addTab(new Widget("Widget 4"), 0);
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
   
   removeWidget = (widgetName) => {
      
   }
	//Render Nav Bar, Widget Display, Console, and Settings
   render() {
      return (
         <div className="gui">
           	<Navbar addWidget = {this.addWidget} removeWidget = {this.removeWidget} />
           	<Console />
            <WidgetDisplay socket={this.state.websocket} windows={this.state.windows}/>
         </div>
      );
   }

   //
   // Creates a new Window that can hold multiple tabs
   //
   createWindow = (tab) => {
      if(tab != null) {
         let newWindow = new Window(this);
         newWindow.addTab(tab);
         this.setState({windows: this.state.windows.push(newWindow)});
      }
   }

   //
   // Creates a new Window that can hold multiple tabs
   //
   addTab = (tab, index) => {
      if(tab != null) {
         this.state.windows[index].addTab(tab);
         this.setState({windows: this.state.windows});
      }
   }

   removeTab = (tab) => {
      if(tab != null) {
         this.state.windows.forEach((item) => {
            item.removeTab(tab);
         });
         this.setState({windows: this.state.windows});
      }
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}


class Window {
   constructor(parent) {
      this.parent = parent;
      this.openTab = 0;
      this.tabs = [];
   }

   addTab(tab) {
      this.tabs.push(tab);
   }

   removeTab(tab) {
      const index = this.tabs.indexOf(tab);
      if (index > -1) {
        this.tabs.splice(index, 1);
        this.openTab -= 1;
        if(this.openTab < 0) {
           this.openTab = 0;
        }
      }
   }

   setOpenTab = (tab) => {
      let i = this.tabs.indexOf(tab);
      if(i >= 0) {
         this.openTab = i;
      }
   }

   render = () => {
      if(this.tabs.length > 0) {
         let window = this;
         return (
            <div className="widgetWindow">
               {this.tabs.map(function(tab) {
                  return (
                        <div className="widgetTab" onClick={() => {window.setOpenTab(tab)}}>
                        {tab.getTitle()}
                        <span onClick={() => window.parent.removeTab(tab)}>    (&#215;)</span>
                     </div>
                  );}
               )}
               {this.tabs[this.openTab].renderWidget()}
            </div>
         );
      }
   }
}

export default GUI;
