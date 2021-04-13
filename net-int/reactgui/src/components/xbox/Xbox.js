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
        if (pressed) {
          document.getElementById("a").style.top = "247px";
          document.getElementById("a").style.opacity = "50%";
        } else {
          document.getElementById("a").style.top = "243px";
          document.getElementById("a").style.opacity = "100%";
        }

      case "B":
        this.setState({
          B: pressed,
        });
        if (pressed) {
          document.getElementById("b").style.top = "194px";
          document.getElementById("b").style.opacity = "50%";
        } else {
          document.getElementById("b").style.top = "187px";
          document.getElementById("b").style.opacity = "100%";
        }
        break;
      case "X":
        this.setState({
          X: pressed,
        });
        if (pressed) {
          document.getElementById("x").style.top = "194px";
          document.getElementById("x").style.opacity = "50%";
        } else {
          document.getElementById("x").style.top = "187px";
          document.getElementById("x").style.opacity = "100%";
        }
        break;
      case "Y":
        this.setState({
          Y: pressed,
        });
        if (pressed) {
          document.getElementById("y").style.top = "139px";
          document.getElementById("y").style.opacity = "50%";
        } else {
          document.getElementById("y").style.top = "135px";
          document.getElementById("y").style.opacity = "100%";
        }
        break;
      case "Start":
        this.setState({
          start: pressed,
        });
        if (pressed) {
          document.getElementById("start").style.opacity = "50%";
        } else {
          document.getElementById("start").style.opacity = "100%";
        }
        break;
      case "Back":
        this.setState({
          back: pressed,
        });
        if (pressed) {
          document.getElementById("back").style.opacity = "50%";
        } else {
          document.getElementById("back").style.opacity = "100%";
        }
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
        if (pressed) {
          document.getElementById("bumpl").style.opacity = "50%";
        } else {
          document.getElementById("bumpl").style.opacity = "100%";
        }
        break;
      case "RB":
        this.setState({
          rb: pressed,
        });
        if (pressed) {
          document.getElementById("bumpr").style.opacity = "50%";
        } else {
          document.getElementById("bumpr").style.opacity = "100%";
        }
        break;
      case "LS":
        this.setState({
          ls: pressed,
        });
        if (pressed) {
          document.getElementById("stick").style.opacity = "50%";
        } else {
          document.getElementById("stick").style.opacity = "100%";
        }
        break;
      case "RS":
        this.setState({
          rs: pressed,
        });
        if (pressed) {
          document.getElementById("stick2").style.opacity = "50%";
        } else {
          document.getElementById("stick2").style.opacity = "100%";
        }
        break;
      case "DPadUp":
        this.setState({
          up: pressed,
        });
        if (pressed) {
          document.getElementById("up").style.opacity = "50%";
        } else {
          document.getElementById("up").style.opacity = "100%";
        }
        break;
      case "DPadDown":
        this.setState({
          down: pressed,
        });
        if (pressed) {
          document.getElementById("down").style.opacity = "50%";
        } else {
          document.getElementById("down").style.opacity = "100%";
        }
        break;
      case "DPadLeft":
        this.setState({
          left: pressed,
        });
        if (pressed) {
          document.getElementById("left").style.opacity = "50%";
        } else {
          document.getElementById("left").style.opacity = "100%";
        }
        break;
      case "DPadRight":
        this.setState({
          right: pressed,
        });
        if (pressed) {
          document.getElementById("right").style.opacity = "50%";
        } else {
          document.getElementById("right").style.opacity = "100%";
        }
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
        });
        document.getElementById("stick").style.left = 98 + value * 20 + "px";
        break;
      case "RightStickX":
        this.setState({
          rsX: value,
        });
        document.getElementById("stick2").style.left = 394 + value * 20 + "px";
        break;
      case "LeftStickY":
        this.setState({
          lsY: value,
        });
        document.getElementById("stick").style.top = 175 + value * -20 + "px";
        break;
      case "RightStickY":
        this.setState({
          rsY: value,
        });
        document.getElementById("stick2").style.top = 296 + value * -20 + "px";
        break;
      case "LeftTrigger":
        this.setState({
          ltAxis: value,
        });
        document.getElementById("lt").style.filter =
          "invert(" + Math.abs(value) + ")";
        break;
      case "RightTrigger":
        this.setState({
          rtAxis: value,
        });
        document.getElementById("rt").style.filter =
          "invert(" + Math.abs(value) + ")";
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
          <img src="/xboxImages/bg.png" id="bg" />
          <img src="/xboxImages/a.png" id="a" />
          <img src="/xboxImages/b.png" id="b" />
          <img src="/xboxImages/x.png" id="x" />
          <img src="/xboxImages/y.png" id="y" />
          <img src="/xboxImages/left.png" id="left" />
          <img src="/xboxImages/right.png" id="right" />
          <img src="/xboxImages/up.png" id="up" />
          <img src="/xboxImages/down.png" id="down" />
          <img src="/xboxImages/stick.png" id="stick" />
          <img src="/xboxImages/stick2.png" id="stick2" />
          <img src="/xboxImages/bumperleft.png" id="bumpl" />
          <img src="/xboxImages/bumperright.png" id="bumpr" />
          <img src="/xboxImages/lt.png" id="lt" />
          <img src="/xboxImages/rt.png" id="rt" />
          <img src="/xboxImages/left.png" id="back" />
          <img src="/xboxImages/right.png" id="start" />
        </div>
      </Gamepad>
    );
  }
}
