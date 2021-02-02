import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
import Settings from "../Settings/Settings.js";
import Controller from "../controller/Controller.js";
import Camera from "../camera/Camera.js";
import TestWidget from "../widgets/TestWidget.js";

import "../widgets/Widget.css";
import "./GUI.css";

class GUI extends React.Component {
   WIDGET_DICT = {
      "settings": <Settings />,
      "camera": <Camera />,
      "widget": <TestWidget />,
      "console": <Console />,
      "controller": <Controller />
   }

   state = {
      websocket: null,
      settings: {},
      windows: []
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');

      this.createWindow("widget");
      this.addTab("camera", 0);
      this.addTab("settings", 0);
      this.addTab("widget", 0);
      this.createWindow("widget");
      this.addTab("camera", 1);
      this.addTab("settings", 1);
      this.addTab("widget", 1);
      //this.addTab(new Widget("Widget 3"), 0);
      //this.addTab(new Widget("Widget 4"), 0);
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
   }
	//Render Nav Bar, Widget Display, Console, and Settings
   render() {
      return (
         <div className="gui">
           	<Navbar addWidget = {this.addWidget} removeWidget = {this.removeWidget} />
           	<Console />

            <div className="widgetDisplay">
               {
                  this.renderWindows()
                  //Render Widgets
               }
            </div>
         </div>
      );
   }


   /////////////////////////////////
   //                             //
   //  Widget Rendering methods   //
   //                             //
   /////////////////////////////////
   renderWindows = () => {
      if(this.state.windows.length > 0) {
         let parent = this;
         return this.state.windows.map((window, index) => {
            console.log(window);
            return parent.renderWindow(window, index);
         });
      }
   }

   renderWindow = (window, index) => {
      if(window.tabs.length > 0) {
         return (
            <div className="widgetWindow">
               {
                  window.tabs.map((tab) => {
                     return (
                        <div className="widgetTab" onClick={() => {this.setOpenTab(tab, index)}}>
                           {tab}
                           <span onClick={() => this.removeTab(tab, index)}>    (&#215;)</span>
                        </div>
                     );
                  })
               }
               {this.getOpenTab(window)}
            </div>
         );
      }
   }


   /////////////////////////////////
   //                             //
   //  Window Managing methods    //
   //                             //
   /////////////////////////////////

   //
   // Creates a new Window that can hold multiple tabs
   //
   createWindow = (tab) => {
      if(tab != null) {
         let newWindow = {openTab: 0, tabs: [tab]}
         this.setState({windows: this.state.windows.push(newWindow)});
      }
   }

   //
   // Creates a new Window that can hold multiple tabs
   //
   addTab = (tab, index) => {
      if(tab != null) {
         this.state.windows[index].tabs.push(tab);
         this.setState({windows: this.state.windows});
      }
   }

   removeTab = (tab, index) => {
      if(tab != null) {
         let w = this.state.windows[index];
         const i = w.tabs.indexOf(tab);
         if (i > -1) {
           w.tabs.splice(i, 1);
           w.openTab -= 1;
           if(w.openTab < 0) {
              w.openTab = 0;
           }
         }
         this.setState({windows: this.state.windows});
      }
   }

   setOpenTab = (tab, index) => {
      if(tab != null) {
         let w = this.state.windows.[index];
         let i = w.tabs.indexOf(tab);
         if(i >= 0) {
            w.openTab = i;
         }
         this.setState({windows: this.state.windows});
         return true;
      }
      return false;
   }

   /////////////////////////////////
   //                             //
   //  Widget Generating methods  //
   //                             //
   /////////////////////////////////

   getOpenTab = (tab) => {
      return (
         <div className="widgetContent">
            {this.WIDGET_DICT[tab.tabs[tab.openTab]]}
         </div>
      );
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}

export default GUI;
