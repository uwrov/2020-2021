import React from 'react';

class TestGui extends React.Component {
   state = {
      cam_ip: "localhost",
      cam_ports: ["8080", "8081"],
      main_cam_index: 0,
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

export default TestGui;
