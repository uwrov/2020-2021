import Component from React;

class MainGUI extends Component {
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
