import React from "react";
import "./Xbox.css";
import Gamepad from "react-gamepad";
import Draggable from "react-draggable";

const socket = require("socket.io-client")("http://localhost:4040");
const AXIS_THROTTLE = 10;

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

  vect = {
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
  }

  camera_index = 0;

  BUTTON_OPACITY = {
    true: "50%",
    false: "100%",
  };

  BUTTON_TOP = {
    true: 7,
    false: 0,
  };

  topOffset = {
    A: 243,
    B: 187 + this.BUTTON_TOP[this.state.B],
    X: 187 + this.BUTTON_TOP[this.state.X],
    Y: 135 + this.BUTTON_TOP[this.state.Y],
    LeftStickY: 175,
    RightStickY: 296
  }

  leftOffset = {
    LeftStickX: 98,
    RightStickX: 394
  }

  constructor(props) {
    super();
    this.handleChange = this.handleChange.bind(this);
    this.handleAxis = this.handleAxis.bind(this);
    console.log("hello there")
  }

  handleChange(buttonName, pressed) {
    console.log(buttonName)
    let change = {};
    change[buttonName] = pressed;
    this.setState(change);
  }

  getTriggerStyle(value) {
    return {
      filter: "invert(" + Math.abs(value) + ")",
    }
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
    let rounded = Math.round(value * AXIS_THROTTLE) / AXIS_THROTTLE
    if(Math.abs(this.state[axisName] - rounded) > 0) {
      console.log(axisName + ", " + rounded);
      let change = {};
      change[axisName] = rounded;
      this.setState(change);
    }
  }

  updateVects() {
    let temp_ang_z = 0;
    if (this.state.LeftTrigger != 0) {
      temp_ang_z = this.state.LeftTrigger;
    } else if (this.state.RightTrigger != 0) {
      temp_ang_z = -1 * this.state.RightTrigger;
    }

    this.vect = {
      lin_x: this.state.LeftStickY,
      lin_y: this.state.LeftStickX,
      lin_z: this.state.RightStickY,
      ang_x: 0,
      ang_y: 0,
      ang_z: temp_ang_z,
    }
  }

  updateCameraIndex() {
      let currIndex = this.camera_index;
      if(this.state.DPadUp) currIndex = 0;
      else if(this.state.DPadRight) currIndex = 1;
      else if(this.state.DPadDown) currIndex = 2;
      else if(this.state.DPadLeft) currIndex = 3;

      if(currIndex != this.camera_index) {
        this.camera_index = currIndex;
        socket.emit("Set Camera", this.camera_index);
      }
  }

  componentDidUpdate() {
    this.updateVects();
    this.updateCameraIndex();
    console.log('sending state');
    socket.emit("Send State", this.vect);
  }

  render() {
    return (
      <Gamepad
        onButtonChange={this.handleChange}
        onAxisChange={this.handleAxis}
      >
        <div className="gamepad">
          <img src="/xboxImages/bg.png" id="bg" />
          <img src="/xboxImages/a.png" id="a" style={
            {
              ...this.getOpacity(this.state.A),
              ...this.getTopOffsetStyle(this.topOffset.A + this.BUTTON_TOP[this.state.A])
            }} />
          <img src="/xboxImages/b.png" id="b" style={
            {
              ...this.getOpacity(this.state.B),
              ...this.getTopOffsetStyle(this.topOffset.B + this.BUTTON_TOP[this.state.B])
            }} />
          <img src="/xboxImages/x.png" id="x" style={
            {
              ...this.getOpacity(this.state.X),
              ...this.getTopOffsetStyle(this.topOffset.X + this.BUTTON_TOP[this.state.X])
            }} />
          <img src="/xboxImages/y.png" id="y" style={
            {
              ...this.getOpacity(this.state.Y),
              ...this.getTopOffsetStyle(this.topOffset.Y + this.BUTTON_TOP[this.state.Y])
            }} />
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
              ...this.getLeftOffsetStyle(this.leftOffset.LeftStickX + this.state.LeftStickX * 20),
              ...this.getTopOffsetStyle(this.topOffset.LeftStickY + this.state.LeftStickY * -20)
            }}
          />{" "}
          <img
            src="/xboxImages/stick2.png"
            id="stick2"
            style={{
              ...this.getLeftOffsetStyle(this.leftOffset.RightStickX + this.state.RightStickX * 20),
              ...this.getTopOffsetStyle(this.topOffset.RightStickY + this.state.RightStickY * -20)
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
            style={this.getTriggerStyle(this.state.LeftTrigger)}
          />
          <img
            src="/xboxImages/rt.png"
            id="rt"
            style={this.getTriggerStyle(this.state.RightTrigger)}
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
