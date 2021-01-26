import React from "react";
import "./Widget.css";

class Widget {

   constructor(title = "Widget") {
      this.title = title;
   }

   //
   // Provide custom renderMethod()
   //
   render() {
      return (
         <div className="emptyWidget">
         </div>
      );
   }

   renderWidget() {
      return (
         <div className="widgetContent">
            {this.render()}
         </div>
      );
   }

   getTitle() {
      return this.title;
   }
}

export default Widget;
