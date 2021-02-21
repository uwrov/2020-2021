import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
import Settings from "../Settings/Settings.js";
import Controller from "../controller/Controller.js";
import RosCamera from "../rosCamera/RosCamera.js";
import IpCamera from "../ipCamera/IpCamera.js";

import TestWidget from "../widgets/TestWidget.js";
import Xbox from "../xbox/Xbox.js";

import "../widgets/Widget.css";
import "./GUI.css";

let WIDGET_DICT = {
   "settings": <Settings />,
   "ros_camera": <RosCamera />,
   "ip_camera": <IpCamera />,
   "widget": <TestWidget />,
   "console": <Console />,
   "controller": <Xbox />
};



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


   /////////////////////////////////
   //                             //
   //  Window Managing methods    //
   //                             //
   /////////////////////////////////
   /**
   //
   // Creates a new Window that can hold multiple tabs
   //
   createWindow = (tab) => {
      if(tab != null) {
         //let newWindow = {openTab: 0, tabs: [tab]}
         let newWindow = new Window();
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

   removeTab = (tab, index) => {
      if(tab != null) {
         let w = this.state.windows[index];
         w.removeTab(tab);

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
         w.setOpenTab(tab);

         let i = w.tabs.indexOf(tab);
         if(i >= 0) {
            w.openTab = i;
         }

         this.setState({windows: this.state.windows});
         return true;
      }
      return false;
   }
   */
   /////////////////////////////////
   //                             //
   //  Widget Generating methods  //
   //                             //
   /////////////////////////////////

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}



class Window {
   constructor() {
      this.hasLeafChildren = false;
      this.child = [];
      this.openTab = 0;
      this.style = {};
   }


   /**
   //
   // Creates a new Window that can hold multiple tabs
   //
   addTab = (tab) => {
      if(tab != null && this.leaf) {
         this.tabs.push(tab);
      }
   }

   addWindow = (window) => {
      if(window instanceof Window.class && !this.leaf) {
         this.windows.push(window);
      } else if(this.tabs.length < 1) {
         this.windows.push(window);
         this.leaf = false;
      } else {
         let temp = new Window();
         temp.tabs = this.tabs;
         temp.openTab = this.openTab;
         this.tabs = [];
         this.windows.push(temp);
         this.leaf = false;
      }
   }

   removeTab = (tab) => {
      if(tab != null && this.leaf) {
         const i = this.tabs.indexOf(tab);
         if (i > -1) {
           this.tabs.splice(i, 1);
           this.openTab -= 1;
           if(this.openTab < 0) {
              this.openTab = 0;
           }
         }
      } else {
         this.windows.forEach((window) => {
            window.removeTab(tab);
         });
      }
   }

   setOpenTab = (tab) => {
      if(tab != null && this.leaf) {
         let i = this.tabs.indexOf(tab);
         if(i >= 0) {
            this.openTab = i;
         }
         return true;
      }
      return false;
   }

   render = () => {
      if(this.tabs.length > 0) {
         return (
            <div className="widgetWindow" style={this.style}>
               {
                  this.tabs.map((tab) => {
                     return (
                        <div className="widgetTab" onClick={() => {this.setOpenTab(tab)}}>
                           {tab}
                           <span onClick={() => this.removeTab(tab)}>    (&#215;)</span>
                        </div>
                     );
                  })
               }
               {this.getOpenTab()}
            </div>
         );
      } else if(!this.leaf && this.windows.length > 0) {
         return (
            <div className="widgetWindow" style={this.style}>
               {
                  this.windows.map((window) => {
                     return window.render();
                  })
               }
            </div>
         )
      }
   }

   getOpenTab = () => {
      if(this.leaf) {
         return (
            <div className="widgetContent">
               {WIDGET_DICT[this.tabs[this.openTab]]}
            </div>
         );
      }
   }
   */
}

class Leaf {
   constructor() {
      this.type = "settings";
   }
}


function add(object, node) {

}

function remove(object, node) {

}

function get(object, windowId, componentId) {
   if(object instanceof Window.class) {
      if(!object.hasLeafChildren) {
         for(let i = 0; i < object.child.length; i++) {
            let obj = get(object.child[i], windowId, componentId);
            if(obj instanceof Leaf.class) {
               return obj;
            } else if (obj instanceof Number) {
               windowId = windowId - obj;
            } else {
               throw Error;
            }
         }
         return null;
      } else {
         if(object.child.length < windowId) {
            return object.child.length - windowId;
         } else {
            return object.child[windowId];
         }
      }
   } else {
      return null;
   }
}

function setTab(object, windowId, tabId) {
   if(object instanceof Window.class && windowId >= 0) {
      if(!object.hasLeafChildren) {
         let winCount = 0;
         for(let i = 0; i < object.child.length; i++) {
            let c = object.child[i];
            let subCount = setTab(c, windowId - winCount, tabId);
            winCount += subCount;
         }
         return winCount;
      } else {
         if(windowId == 0) {
            object.openTab = tabId;
         }
         return 1;
      }
   }
   return 0;
}

function renderWindows(object) {
   return (
      <div>
         {object.child.map((child) => {
            if(child instanceof Window.class) {
               if(child.hasLeafChildren) {

               } else {
                  return (
                     <div>
                     </div>
                  );
               }
            } else {
               return generateComponent(child);
            }
         })}
      </div>
   );
}

function generateComponent(component) {
   return null;
}
export default GUI;
