import Node from './node.js';
import React from 'react';
import './key.css';
const socket = require('socket.io-client')('http://localhost:4040');


export default class key extends React.Component {
   constructor() {
      super();
      this.state = {
         left: 0,
         up: 0,
         down: 0,
         right: 0
      };

      document.addEventListener("keydown", event => {
         if(event.keyCode === 65){
            console.log('you pressed left')
            this.setState({
               left: 1
            });
         }
         if(event.keyCode === 87){
            this.setState({
               up: 1
            });
         }
         if(event.keyCode === 83){
            this.setState({
               down: 1
            });
         }
         if(event.keyCode === 68){
            this.setState({
               right: 1
            });
         }
         socket.emit("Send State", this.state);
      });

      document.addEventListener("keyup", event => {
         if(event.keyCode === 65){
            this.setState({
               left: 0
            });
         }
         if(event.keyCode === 87){
            this.setState({
               up: 0
            });
         }
         if(event.keyCode === 83){
            this.setState({
               down: 0
            });
         }
         if(event.keyCode === 68){
            this.setState({
               right: 0
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
               display={this.state.up ? "pressed" : "not"} />), 
            (<Node display="hidden"/>)
      ];
      
      let bottomArrows = [
      (<Node 
         id="left" 
         display={this.state.left ? "pressed" : "not"} />), 
      (<Node 
         id="down" 
         display={this.state.down ? "pressed" : "not"} />), 
      (<Node 
         id="right" 
         display={this.state.right ? "pressed" : "not"} />)];
      return(
         <div className="key">
            <div>{topArrows}</div>
            <div>{bottomArrows}</div>
         </div>
      );
   }
}
