import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
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

	//Render Nav Bar, Widget Display, Console, and Settings
   render() {
      return (
         <div className="gui">
           	<Navbar />
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
         this.setState({windows: this.state.windows[index].addTab(tab)});
      }
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}


class Window {
   constructor() {
      this.openTab = 0;
      this.tabs = [];
   }

   addTab(tab) {
      this.tabs.push(tab);
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
