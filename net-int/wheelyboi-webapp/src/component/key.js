import Node from './node.js';
import React from 'react';
import './key.css';
const socket = require('socket.io-client')('http://localhost:4040');

// This component is a visual representation of the movement vector
// that is being sent to the server by the controller.
export default class key extends React.Component {
   // Initialize the output vectors to zero.
   constructor() {
      super();
      this.state = {
         lin_x: 0,
         lin_y: 0,
         lin_z: 0
      };

      // This event listener will be triggered everytime up, down, 
      // left, right, space, and/or shift is being pressed. It will
      // send the current state of the movement vector to the server.
      document.addEventListener("keydown", event => { 
         // Pressed left('a')
         if(event.keyCode === 65){
            this.setState({
               lin_x: -1
            });
         }

         // Pressed up
         if(event.keyCode === 87){
            this.setState({
               lin_y: 1
            });
         }

         // Pressed down
         if(event.keyCode === 83){
            this.setState({
               lin_y: -1
            });
         }

         // Pressed right
         if(event.keyCode === 68){
            this.setState({
               lin_x: 1
            });
         }

         // Pressed space
         if(event.keyCode === 32){
            this.setState({
               lin_z: 1
            });
         }

         // Pressed shift
         if(event.keyCode === 16){
            this.setState({
               lin_z: -1
            });
         }
         socket.emit("Send State", this.state);
      });

      document.addEventListener("keyup", event => {
         if(event.keyCode === 65 || event.keyCode === 68){
            this.setState({
               lin_x: 0
            });
         }
         if(event.keyCode === 87 || event.keyCode === 83){
            this.setState({
               lin_y: 0
            });
         }
         if(event.keyCode === 32 || event.keyCode === 16){
            this.setState({
               lin_z: 0
            });
         }
         socket.emit("Send State", this.state);
      });
   }


   render() {
      // The upper half of the array of Nodes.
      let topArrows = [
            (<Node display="hidden"/>), 
            (<Node 
               id="front" 
               display={this.state.lin_y === 1 ? "pressed" : "not"} />), 
            (<Node display="hidden"/>)
      ];
      
      // The bottom half of the array of Nodes.
      let bottomArrows = [
      (<Node 
         id="left" 
         display={this.state.lin_x === -1 ? "pressed" : "not"} />), 
      (<Node 
         id="back" 
         display={this.state.lin_y === -1 ? "pressed" : "not"} />), 
      (<Node 
         id="right" 
         display={this.state.lin_x === 1 ? "pressed" : "not"} />)];
      return(
         <div className="key">
            <div>
               <div>{topArrows}</div>
               <div>{bottomArrows}</div>
            </div>
            <div>
               <Node 
                  id="up"
                  display={this.state.lin_z === 1 ? "pressed" : "not"} />
               <Node 
                  id="down"
                  display={this.state.lin_z === -1 ? "pressed" : "not"} />
            </div>
         </div>
      );
   }
}
