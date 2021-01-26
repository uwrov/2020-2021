import React from "react";
import Widget from "./Widget.js";
import "./WidgetDisplay.css";

class WidgetDisplay extends React.Component {
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

   renderWindows = () => {
      if(this.props.windows.length > 0) {
         return this.props.windows.map((window) => window.render());
      };
   }
}


export default WidgetDisplay;
