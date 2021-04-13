import Node from "./node.js";
import React from "react";
import "./Controller.css";
import Draggable from "react-draggable";

const socket = require("socket.io-client")("http://localhost:4041");

// This component is a visual representation of the movement vector
// that is being sent to the server by the controller.
export default class Controller extends React.Component {
  // Initialize the output vectors to zero.
  constructor(props) {
    super();
    this.state = {
      lin_x: 0,
      lin_y: 0,
      lin_z: 0,
      config: {
        front: 87,
        back: 83,
        left: 65,
        right: 68,
        up: 84,
        down: 71,
      }
    };

    // This event listener will be triggered everytime up, down,
    // left, right, space, and/or shift is being pressed. It will
    // send the current state of the movement vector to the server.
    document.addEventListener("keydown", (event) => {
      // Pressed left('a')
      if (event.keyCode === this.state.config.left) {
        this.setState({
          lin_x: -1,
        });
        socket.emit("Send State", this.state);
      }

      // Pressed up
      if (event.keyCode === this.state.config.front) {
        this.setState({
          lin_y: 1,
        });
        socket.emit("Send State", this.state);
      }

      // Pressed down
      if (event.keyCode === this.state.config.back) {
        this.setState({
          lin_y: -1,
        });
        socket.emit("Send State", this.state);
      }

      // Pressed right
      if (event.keyCode === this.state.config.right) {
        this.setState({
          lin_x: 1,
        });
        socket.emit("Send State", this.state);
      }

      // Pressed space
      if (event.keyCode === this.state.config.up) {
        this.setState({
          lin_z: 1,
        });
        socket.emit("Send State", this.state);
      }

      // Pressed shift
      if (event.keyCode === this.state.config.down) {
        this.setState({
          lin_z: -1,
        });
        socket.emit("Send State", this.state);
      }
      //console.log(this.state);

    });

    document.addEventListener("keyup", (event) => {
      if (
        event.keyCode === this.state.config.left ||
        event.keyCode === this.state.config.right
      ) {
        this.setState({
          lin_x: 0,
        });
        socket.emit("Send State", this.state);
      }
      if (
        event.keyCode === this.state.config.back ||
        event.keyCode === this.state.config.front
      ) {
        this.setState({
          lin_y: 0,
        });
        socket.emit("Send State", this.state);
      }
      if (
        event.keyCode === this.state.config.up ||
        event.keyCode === this.state.config.down
      ) {
        this.setState({
          lin_z: 0,
        });
        socket.emit("Send State", this.state);
      }
      //console.log(this.state);

    });
  }

  render() {
    // The upper half of the array of Nodes.
    let topArrows = [
      <Node display="hidden" />,
      <Node id="front" display={this.state.lin_y === 1 ? "pressed" : "not"} />,
      <Node display="hidden" />,
      <Node display="hidden" />,
      <Node id="up" display={this.state.lin_z === 1 ? "pressed" : "not"} />,
    ];

    // The bottom half of the array of Nodes.
    let bottomArrows = [
      <Node id="left" display={this.state.lin_x === -1 ? "pressed" : "not"} />,
      <Node id="back" display={this.state.lin_y === -1 ? "pressed" : "not"} />,
      <Node id="right" display={this.state.lin_x === 1 ? "pressed" : "not"} />,
      <Node display="hidden" />,
      <Node id="down" display={this.state.lin_z === -1 ? "pressed" : "not"} />,
    ];
    return (
      <Draggable>
        <div className="key">
          <div>
            <div>{topArrows}</div>
            <div>{bottomArrows}</div>
          </div>
        </div>
      </Draggable>
    );
  }
}
