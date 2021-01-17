import React from "react";
import "./WidgetDisplay.css";

class WidgetDisplay extends React.Component {
   state = {
      windows: [
      ]
   }

   constructor(props) {
      super(props);

   }

   render() {
      return (
         <div className="widgetDisplay">
            {
               this.renderWindows()
               //Render Widgets
            }
         </div>
      );
   }

   createWindow = (tab) => {
      if(tab != null) {
         let newWindow = {
            openTab: 0,
            tabs: [tab]
         }
         this.setState({windows: this.state.windows.append(newWindow)});
      }
   }

   renderWindows = () => {
      return (
         this.state.windows.map((window) => window.render())
      );
   }
}

class Window {
   constructor() {
      this.openTab = 0;
      this.tabs = [];
   }

   openTab(tab) {
      let i = this.tabs.indexOf();
   }

   render() {
      return (
         <div>
            {this.tabs.map((tab) => tab.renderTab())}
            {window.tabs[window.openTab].renderWidget()}
         </div>
      );
   }
}

export default WidgetDisplay;
