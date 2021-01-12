import React from "react";

class WidgetDisplay extends React.Component {
   state = {
      windows: [
         [CameraWidget]
         [ControllerWidget, ImageWidget]
      ]
   }

   constructor(props) {
      super(props);

   }

   render() {
      return (
         <div>
            {
               //Render Widgets
            }
         </div>
      );
   }
}

export default WidgetDisplay;
