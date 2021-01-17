import React from "react";
import "./Widget.css";

class Widget {
   constructor() {

   }

   render() {
      return ();
   }
   
   renderWidget() {
      return (
         <div className="widgetContent">
            {render()}
         </div>
      );
   }

   renderTab = () => {
      return (
         <div className="widgetTab">Widget</div>
      );
   }
}

export default Widget;
