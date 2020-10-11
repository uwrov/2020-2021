import React from 'react';
import Node from './node.js';
import './key.css';
socket = require('socket.io-client')('http://localhost:4000');

export default class key extends React.Component {
   constructor() {
      super();
      this.state = {
         left: false,
         up: false,
         down: false,
         right: false
      };

      socket.on('sendState', function(){
         socket.emit(this.state);
      });

      document.addEventListener("keydown", event => {
         if(event.keyCode === 65){
            this.setState({
               left: true
            });
         }
         if(event.keyCode === 87){
            this.setState({
               up: true
            });
         }
         if(event.keyCode === 83){
            this.setState({
               down: true
            });
         }
         if(event.keyCode === 68){
            this.setState({
               right: true
            });
         }
      });

      document.addEventListener("keyup", event => {
         if(event.keyCode === 65){
            this.setState({
               left: false
            });
         }
         if(event.keyCode === 87){
            this.setState({
               up: false
            });
         }
         if(event.keyCode === 83){
            this.setState({
               down: false
            });
         }
         if(event.keyCode === 68){
            this.setState({
               right: false
            });
         }
      });
   }


   render() {
      let arrows = [
         [
            <Node className="hidden"/>, 
            <Node 
               id="up" 
               className={this.state.up ? "pressed" : "not"} />, 
            <Node className="hidden"/>
         ],
         [
            <Node 
               id="left" 
               className={this.state.left ? "pressed" : "not"} />, 
            <Node 
               id="down" 
               className={this.state.down ? "pressed" : "not"} />, 
            <Node 
               id="right" 
               className={this.state.right ? "pressed" : "not"} />]
      ];
      return(
         <div className="key">
            <div>{arrows}</div>
         </div>
      );
   }
}