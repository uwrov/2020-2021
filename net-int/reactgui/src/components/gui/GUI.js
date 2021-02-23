import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';

import "../widgets/Widget.css";
import "./GUI.css";

import { Window, Leaf, add, remove, get, setTab } from "../../datastructs/WidgetTree.js";

class GUI extends React.Component {

   state = {
      websocket: null,
      settings: {},
      windows: []
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');

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
            return window.render();
         });
      }
   }

   renderWindow = (window, index) => {
      if(window.tabs.length > 0) {
         return (
            <div className="widgetWindow" style={window.style}>
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

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}

export default GUI;
