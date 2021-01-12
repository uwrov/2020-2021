import React from "react";
import Navbar from "../navbar/NavBar.js";

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
            	//<Navbar/>
               //Render Nav Bar
               //Render Widget Display
               //Render Console
               //Render settings
            }
         </div>
      );
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }
}

export default GUI;
