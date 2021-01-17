import React from "react";
import Navbar from "../navbar/NavBar.js";
import Console from '../console/Console.js';
import WidgetDisplay from "../widgets/WidgetDisplay.js";
import "./GUI.css";

class GUI extends React.Component {
   state = {
      websocket: null,
      settings: {},
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');
   }

	//Render Nav Bar, Widget Display, Console, and Settings
   render() {
      return (
         <div>
        	<Navbar />
        	<Console />   
         </div>
      );
   }

   updateSettings = (newSettings) => {
      this.setState({ settings: newSettings });
   }

}

export default GUI;
