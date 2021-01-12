import React from "react";
import Console from "../console/Console"
class GUI extends React.Component {
   state = {
      websocket: null,
      settings: {}
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');
   }

   render() {
      return (
         <div>
            {
               //Render Nav Bar
               //Render Widget Display
               //Render Console
               //Render settings
               <Console />
            }
         </div>
      );
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}

export default GUI;