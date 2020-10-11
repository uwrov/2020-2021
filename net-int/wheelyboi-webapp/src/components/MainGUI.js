import React from "react";

class MainGUI extends React.Component {
   state = {
   }

   constructor(props) {
      super(props);

      this.state.websocket = require('socket.io-client')('http://localhost:4040');
   }

   render() {
      return (
         <div>
         </div>
      );
   }
}

export default MainGUI;
