import React from "react";
import "./Xbox.css";
import Gamepad from "react-gamepad";
import Draggable from "react-draggable";

const socket = require("socket.io-client")("http://localhost:4041");

export default class Xbox extends React.Component {
  state = {
    A: false,
    B: false,
    X: false,
    Y: false,
    Start: false,
    Back: false,
    LT: false,
    RT: false,
    LB: false,
    RB: false,
    LS: false,
    RS: false,
    DPadUp: false,
    DPadDown: false,
    DPadLeft: false,
    DPadRight: false,
    RightTrigger: 0,
    LeftTrigger: 0,
    LeftStickX: 0,
    LeftStickY: 0,
    RightStickX: 0,
    RightStickY: 0,
  };

  vect: {
    lin_x: 0,
    lin_y: 0,
    lin_z: 0,
    ang_x: 0,
    ang_y: 0,
    ang_z: 0
  }

  BUTTON_OPACITY = {
    true: "50%",
    false: "100%",
  };

  BUTTON_TOP = {
    true: 7,
    false: 0,
  };

  topOffset = {
    A: 243 + this.BUTTON_TOP[this.state.A],
    B: 187 + this.BUTTON_TOP[this.state.B],
    X: 187 + this.BUTTON_TOP[this.state.X],
    Y: 135 + this.BUTTON_TOP[this.state.Y],
    LeftStickY: 175 + this.state.LeftStickY * -20,
    RightStickY: 296 + this.state.RightStickY * -20
  }

  leftOffset = {
    LeftStickX: 98 + this.state.LeftStickX * 20,
    RightStickX: 394 + this.state.RightStickX * 20
  }

  style = {
    LeftTrigger: {
      filter: "invert(" + Math.abs(this.state.LeftTrigger) + ")",
    },
    RightTrigger: {
      filter: "invert(" + Math.abs(this.state.RightTrigger) + ")",
    }
  };
  // TODO
  // document.addEventListener("keydown", (event) => {
  //   switch (event.keyCode) {
  //     // Pressed left('a')
  //     case this.state.config.left:
  //       this.setState({
  //         lin_x: -1,
  //       });
  //       socket.emit("Send State" , this.state);
  //     // Pressed up
  //     case this.state.config.front:
  //       this.setState({
  //         lin_y: 1,
  //       });
  //       socket.emit("Send State", this.state);
  //     // Pressed down
  //     case this.state.config.back:
  //       this.setState({
  //         lin_y: -1,
  //       });
  //       socket.emit("Send State", this.state);
  //     // Pressed right
  //     case this.state.config.right:
  //       this.setState({
  //         lin_x: 1,
  //       });
  //       socket.emit("Send State", this.state);
  //     // Pressed space
  //     case this.state.config.up:
  //       this.setState({
  //         lin_z: 1,
  //       });
  //       socket.emit("Send State", this.state);
  //     // Pressed shift
  //     case this.state.config.down:
  //       this.setState({
  //         lin_z: -1,
  //       });
  //       socket.emit("Send State", this.state);
  //     default:
  //       console.log("Error: keyCode not defined");
  //     }
  // }
  constructor(props) {
    super();
    this.handleChange = this.handleChange.bind(this);
    this.handleAxis = this.handleAxis.bind(this);
    console.log("hello there")
  }

  handleChange(buttonName, pressed) {
    console.log("hello there")
    let change = {};
    change[buttonName] = pressed;
    this.setState(change);
  }

  getTopOffsetStyle(value) {
    return {top: value + "px"}
  }

  getLeftOffsetStyle(value) {
    return {left: value + "px"}
  }

  getOpacity(button) {
    return {opacity: this.BUTTON_OPACITY[button]}
  }

  handleAxis(axisName, value, previousValue) {
    let change = {};
    change[axisName] = value;
    this.setState(change)
  }

  updateVects() {
    let temp_ang_z = 0;
    if (this.state.DPadLeft) {
      temp_ang_z = -1;
    } else if (this.state.DPadRight) {
      temp_ang_z = 1;
    }
    this.vect: {
      lin_x: this.state.LeftStickX,
      lin_y: this.state.LeftStickY,
      lin_z: this.state.RightStickY,
      ang_x: 0,
      ang_y: 0,
      ang_z: temp_ang_z,
    }
  }

  componentDidUpdate() {
    console.log("Sending: ");
    console.log(this.vect);
    updateVects();
    socket.emit("Send State", this.state.vect);
  }

  render() {
    return (
      <Gamepad
        onButtonChange={this.handleChange}
        onAxisChange={this.handleAxis}
      >
        <div className="gamepad">
          <img src="/xboxImages/bg.png" id="bg" />
          <img src="/xboxImages/a.png" id="a" style={this.getOpacity(this.state.A)} />
          <img src="/xboxImages/b.png" id="b" style={this.getOpacity(this.state.B)} />
          <img src="/xboxImages/x.png" id="x" style={this.getOpacity(this.state.X)} />
          <img src="/xboxImages/y.png" id="y" style={this.getOpacity(this.state.Y)} />
          <img
            src="/xboxImages/left.png"
            id="left"
            style={this.getOpacity(this.state.DPadLeft)}
          />
          <img
            src="/xboxImages/right.png"
            id="right"
            style={this.getOpacity(this.state.DPadRight)}
          />
          <img src="/xboxImages/up.png" id="up" style={this.getOpacity(this.state.DPadUp)} />
          <img
            src="/xboxImages/down.png"
            id="down"
            style={this.getOpacity(this.state.DPadDown)}
          />
          <img
            src="/xboxImages/stick.png"
            id="stick"
            style={{
              ...this.getLeftOffsetStyle(this.leftOffset.LeftStickX),
              ...this.getTopOffsetStyle(this.topOffset.LeftStickY)
            }}}
          />{" "}
          <img
            src="/xboxImages/stick2.png"
            id="stick2"
            style={{
              ...this.getLeftOffsetStyle(this.leftOffset.RightStickX),
              ...this.getTopOffsetStyle(this.topOffset.RightStickY)
            }}
          />{" "}
          <img
            src="/xboxImages/bumperleft.png"
            id="bumpl"
            style={this.getOpacity(this.state.LB)}
          />
          <img
            src="/xboxImages/bumperright.png"
            id="bumpr"
            style={this.getOpacity(this.state.RB)}
          />
          <img
            src="/xboxImages/lt.png"
            id="lt"
            style={this.style.LeftTrigger}
          />
          <img
            src="/xboxImages/rt.png"
            id="rt"
            style={this.style.RightTrigger}
          />
          <img src="/xboxImages/left.png" id="back" style={this.getOpacity(this.state.Back)} />
          <img
            src="/xboxImages/right.png"
            id="start"
            style={this.getOpacity(this.state.Start)}
          />
        </div>
      </Gamepad>
    );
  }
}
