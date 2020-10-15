import Node from './node.js';
import React from 'react';
import './key.css';
const socket = require('socket.io-client')('http://localhost:4040');


export default class key extends React.Component {
   constructor() {
      super();
      this.state = {
         lin_x: 0,
         lin_y: 0,
         lin_z: 0
      };



      document.addEventListener("keydown", event => {
         if(event.keyCode === 65){
            console.log('you pressed left');
            this.setState({
               lin_x: -1
            });
         }
         if(event.keyCode === 87){
            console.log("you pressed up");
            this.setState({
               lin_y: 1
            });
         }
         if(event.keyCode === 83){
            console.log("you pressed down");
            this.setState({
               lin_y: -1
            });
         }
         if(event.keyCode === 68){
            console.log("you pressed right");
            this.setState({
               lin_x: 1
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
         socket.emit("Send State", this.state);
      });
   }


   render() {
      let topArrows = [
            (<Node display="hidden"/>), 
            (<Node 
               id="up" 
               display={this.state.lin_y === 1 ? "pressed" : "not"} />), 
            (<Node display="hidden"/>)
      ];
      
      let bottomArrows = [
      (<Node 
         id="left" 
         display={this.state.lin_x === -1 ? "pressed" : "not"} />), 
      (<Node 
         id="down" 
         display={this.state.lin_y === -1 ? "pressed" : "not"} />), 
      (<Node 
         id="right" 
         display={this.state.lin_x === 1 ? "pressed" : "not"} />)];
      return(
         <div className="key">
            <div>{topArrows}</div>
            <div>{bottomArrows}</div>
            <div>
               <input type="text" onChange={event => {
                  let newZ = parseFloat(event.target.value);
                  if(!isNaN(newZ)) {
                     this.setState({
                        lin_z: newZ
                     });
                     console.log("this.state.lin_z = " + this.state.lin_z);
                  } else {
                     console.log("that's not a valid float");
                  }
               }}/>
            </div>
         </div>
      );
   }
}
