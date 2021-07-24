import Node from "./node.js";
import React from "react";
import "./Controller.css";
import Draggable from "react-draggable";

const socket = require("socket.io-client")("http://localhost:4040");

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
      ang_x: 0,
      ang_y: 0,
      ang_z: 0,
      a: 0,
      b: 0,
      x: 0,
      y: 0,
      config: {
        front: "w",
        back: "s",
        left: "a",
        right: "d",
        up: "r",
        down: "f",
      },
    };

    // This event listener will be triggered everytime up, down,
    // left, right, space, and/or shift is being pressed. It will
    // send the current state of the movement vector to the server.
    document.addEventListener("keydown", (event) => {
      switch (event.key) {
        // Pressed left('a')
        case this.state.config.left:
          this.setState({
            lin_x: -1,
          });
          break;
        // Pressed up
        case this.state.config.front:
          this.setState({
            lin_y: 1,
          });
          break;
        // Pressed down
        case this.state.config.back:
          this.setState({
            lin_y: -1,
          });
          break;
        // Pressed right
        case this.state.config.right:
          this.setState({
            lin_x: 1,
          });
          break;
        // Pressed space
        case this.state.config.up:
          this.setState({
            lin_z: 1,
          });
          break;
        // Pressed shift
        case this.state.config.down:
          this.setState({
            lin_z: -1,
          });
          break;
        default:
          console.log("Error: key not defined " + event.key);
          break;
      }
      console.log("key down" + event.key);
    });

    document.addEventListener("keyup", (event) => {
      if (
        event.key === this.state.config.left ||
        event.key === this.state.config.right
        /*event.key === this.state.config.back ||
        event.key === this.state.config.front ||
	event.key === this.state.config.up ||
        event.key === this.state.config.down*/
      ) {
        this.setState({
          lin_x: 0,
          lin_y: 0,
          lin_z: 0,
        });
      }
      if (
        event.key === this.state.config.back ||
        event.key === this.state.config.front
      ) {
        this.setState({
          lin_y: 0,
        });
      }
      if (
        event.key === this.state.config.up ||
        event.key === this.state.config.down
      ) {
        this.setState({
          lin_z: 0,
        });
      }
      console.log("key up" + event.key);
    });
  }

  componentDidUpdate() {
    socket.emit("Send State", this.state);
    console.log(this.state + " emitting");
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
