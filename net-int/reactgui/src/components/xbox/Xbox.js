import React from "react";
import "./Xbox.css";
import Gamepad from "react-gamepad";
import Draggable from "react-draggable";

const socket = require("socket.io-client")("http://localhost:4040");

export default class Xbox extends React.Component {
  constructor(props) {
    super();
    this.state = {
      A: false,
      B: false,
      X: false,
      Y: false,
      start: false,
      back: false,
      lt: false,
      rt: false,
      lb: false,
      rb: false,
      ls: false,
      rs: false,
      up: false,
      down: false,
      left: false,
      right: false,
      rtAxis: 0,
      ltAxis: 0,
      lsX: 0,
      lsY: 0,
      rsX: 0,
      rsY: 0,
      vect: {
        lin_x: 0,
        lin_y: 0,
        lin_z: 0,
      },
      stick: "",
      stick2: ""
    };

    BUTTON_OPACITY = {
      true: "50%",
      false: "100%"
    };

    BUTTON_TOP = {
      true: 7,
      false: 0
    };

    BUTTON_LEFT = {

    }

    style = {
      A: {
        top: 243 + BUTTON_TOP[this.state.A] + "px",
        opacity: BUTTON_OPACITY[this.state.A]
      },
      B: {
        top: 187 + BUTTON_TOP[this.state.B] + "px",
        opacity: BUTTON_OPACITY[this.state.B]
      },
      X: {
        top: 187 + BUTTON_TOP[this.state.X] + "px",
        opacity: BUTTON_OPACITY[this.state.X]
      },
      Y: {
        top: 135 + BUTTON_TOP[this.state.Y] + "px",
        opacity: BUTTON_OPACITY[this.state.Y]
      },
      start: {
        opacity: BUTTON_OPACITY[this.state.start]
      },
      back: {
        opacity: BUTTON_OPACITY[this.state.back]
      },
      lb:{
        opacity: BUTTON_OPACITY[this.state.lb]
      },
      rb:{
        opacity: BUTTON_OPACITY[this.state.rb]
      },
      ls:{
        opacity: BUTTON_OPACITY[this.state.ls]
      },
      rs:{
        opacity: BUTTON_OPACITY[this.state.rs]
      },
      up:{
        opacity: BUTTON_OPACITY[this.state.up]
      },
      down:{
        opacity: BUTTON_OPACITY[this.state.down]
      },
      left:{
        opacity: BUTTON_OPACITY[this.state.left]
      },
      right:{
        opacity: BUTTON_OPACITY[this.state.right]
      },
      lsX:{
        left: 98 + this.state.lsX * 20 + "px"
      },
      rsX:{
        left: 394 + this.state.rsX * 20 + "px"
      },
      lsY:{
        top : 175 + this.state.lsY * -20 + "px"
      },
      rsY:{
        top : 296 + this.state.rsY * -20 + "px"
      },
      lt: {
        filter : "invert(" + Math.abs(this.state.lt) + ")"
      },
      rt: {
        filter : "invert(" + Math.abs(this.state.rt) + ")"
      },
      stick:{
        left: this.state.stick
      },
      stick2:{
        top: this.state.stick2
      }

    };

    this.handleChange = this.handleChange.bind(this);
    this.handleAxis = this.handleAxis.bind(this);
  }
  handleChange(buttonName, pressed) {
    switch (buttonName) {
      case "A":
        this.setState({
          A: pressed,
        });
        break;
      case "B":
        this.setState({
          B: pressed,
        });
        break;
      case "X":
        this.setState({
          X: pressed,
        });
        break;
      case "Y":
        this.setState({
          Y: pressed,
        });
        break;
      case "Start":
        this.setState({
          start: pressed,
        });
        break;
      case "Back":
        this.setState({
          back: pressed,
        });
        break;
      case "LT":
        this.setState({
          lt: pressed,
        });
        break;
      case "RT":
        this.setState({
          rt: pressed,
        });
        break;
      case "LB":
        this.setState({
          lb: pressed,
        });
        break;
      case "RB":
        this.setState({
          rb: pressed,
        });
        break;
      case "LS":
        this.setState({
          ls: pressed,
        });
        break;
      case "RS":
        this.setState({
          rs: pressed,
        });
        break;
      case "DPadUp":
        this.setState({
          up: pressed,
        });
        break;
      case "DPadDown":
        this.setState({
          down: pressed,
        });
        break;
      case "DPadLeft":
        this.setState({
          left: pressed,
        });
        break;
      case "DPadRight":
        this.setState({
          right: pressed,
        });
        break;
      default:
        console.log("Error: buttonName not defined");
    }
    //socket.emit("Send State", this.state.vect);
  }

  handleAxis(axisName, value, presiousValue) {
    switch (axisName) {
      case "LeftStickX":
        this.setState({
          lsX: value,
          stick: 98 + value * 20 + "px"
        });
        // document.getElementById("stick").style.left = 98 + value * 20 + "px";
        break;
      case "RightStickX":
        this.setState({
          rsX: value,
          stick2: 394 + value * 20 + "px"
        });
        // document.getElementById("stick2").style.left = 394 + value * 20 + "px";
        break;
      case "LeftStickY":
        this.setState({
          lsY: value,
          stick: 175 + value * -20 + "px";
        });
        // document.getElementById("stick").style.top = 175 + value * -20 + "px";
        break;
      case "RightStickY":
        this.setState({
          rsY: value,
          stick2:  296 + value * -20 + "px
        });
        // document.getElementById("stick2").style.top = 296 + value * -20 + "px";
        break;
      case "LeftTrigger":
        this.setState({
          ltAxis: value,
        });
        break;
      case "RightTrigger":
        this.setState({
          rtAxis: value,
        });
        break;
      default:
        console.log("Error: axisName not defined")
    }
    this.setState({
      vect: {
        lin_x: this.state.lsX,
        lin_y: this.state.lsY,
        lin_z: this.state.rsY
      }
    });
    console.log(this.state.vect);
    socket.emit("Send State", this.state.vect);
  }

  render() {
    return (
      <Gamepad
        onButtonChange={this.handleChange}
        onAxisChange={this.handleAxis}
      >
        <div className="gamepad">
        // TODO: FILL IN THE REST FIX style= this.style.A... does not work
          <img src="/xboxImages/bg.png" id="bg" />
          <img src="/xboxImages/a.png" id="a" style={this.style.A}/>
          <img src="/xboxImages/b.png" id="b" style={this.style.B}/>
          <img src="/xboxImages/x.png" id="x" style={this.style.X}/>
          <img src="/xboxImages/y.png" id="y" style={this.style.Y}/>
          <img src="/xboxImages/left.png" id="left" style={this.style.left}/>
          <img src="/xboxImages/right.png" id="right" style={this.style.right}/>
          <img src="/xboxImages/up.png" id="up" style={this.style.up}/>
          <img src="/xboxImages/down.png" id="down" style={this.style.down}/>
          <img src="/xboxImages/stick.png" id="stick" style={this.style.stick}/> //!
          <img src="/xboxImages/stick2.png" id="stick2" style={this.style.stick2}/> //!
          <img src="/xboxImages/bumperleft.png" id="bumpl" style={this.style.lb}/>
          <img src="/xboxImages/bumperright.png" id="bumpr" style={this.style.rb}/>
          <img src="/xboxImages/lt.png" id="lt" style={this.style.lt}/>
          <img src="/xboxImages/rt.png" id="rt" style={this.style.rt}/>
          <img src="/xboxImages/left.png" id="back" style={this.style.back}/>
          <img src="/xboxImages/right.png" id="start" style={this.style.start}/>
        </div>
      </Gamepad>
    );
  }
}
