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
    // look into move server to make sure they match what move server wants
    vect: {
      lin_x: 0,
      lin_y: 0,
      lin_z: 0,
      ang_x: 0,
      ang_y: 0,
      ang_z: 0,
    },
    stick: {
      left: "",
      top: "",
    },
    stick2: {
      left: "",
      top: "",
    },
  };
  // KEYBOARD_BINDINGS = {
  //   "r": "A"
  // }
  BUTTON_OPACITY = {
    true: "50%",
    false: "100%",
  };

  BUTTON_TOP = {
    true: 7,
    false: 0,
  };

  style = {
    A: {
      top: 243 + this.BUTTON_TOP[this.state.A] + "px",
      opacity: this.BUTTON_OPACITY[this.state.A],
    },
    B: {
      top: 187 + this.BUTTON_TOP[this.state.B] + "px",
      opacity: this.BUTTON_OPACITY[this.state.B],
    },
    X: {
      top: 187 + this.BUTTON_TOP[this.state.X] + "px",
      opacity: this.BUTTON_OPACITY[this.state.X],
    },
    Y: {
      top: 135 + this.BUTTON_TOP[this.state.Y] + "px",
      opacity: this.BUTTON_OPACITY[this.state.Y],
    },
    Start: {
      opacity: this.BUTTON_OPACITY[this.state.Start],
    },
    Back: {
      opacity: this.BUTTON_OPACITY[this.state.Back],
    },
    LB: {
      opacity: this.BUTTON_OPACITY[this.state.LB],
    },
    RB: {
      opacity: this.BUTTON_OPACITY[this.state.RB],
    },
    LS: {
      opacity: this.BUTTON_OPACITY[this.state.LS],
    },
    RS: {
      opacity: this.BUTTON_OPACITY[this.state.RS],
    },
    DPadUp: {
      opacity: this.BUTTON_OPACITY[this.state.DPadUp],
    },
    DPadDown: {
      opacity: this.BUTTON_OPACITY[this.state.DPadDown],
    },
    DPadLeft: {
      opacity: this.BUTTON_OPACITY[this.state.DPadLeft],
    },
    DPadRight: {
      opacity: this.BUTTON_OPACITY[this.state.DPadRight],
    },
    LeftStickX: {
      left: 98 + this.state.LeftStickX * 20 + "px",
    },
    RightStickX: {
      left: 394 + this.state.RightStickX * 20 + "px",
    },
    LeftStickY: {
      top: 175 + this.state.LeftStickY * -20 + "px",
    },
    RightStickY: {
      top: 296 + this.state.RightStickY * -20 + "px",
    },
    LeftTrigger: {
      filter: "invert(" + Math.abs(this.state.LeftTrigger) + ")",
    },
    RightTrigger: {
      filter: "invert(" + Math.abs(this.state.RightTrigger) + ")",
    },
    stick: {
      left: this.state.stick.left,
      top: this.state.stick.top,
    },
    stick2: {
      left: this.state.stick2.left,
      top: this.state.stick2.top,
    },
  };
  // TODO
  // document.addEventListener("keydown", (event) => {
  //   switch (event.keyCode) {
  //     // Pressed left('a')
  //     case this.state.config.left:
  //       this.setState({
  //         lin_x: -1,
  //       });
  //       socket.emit("Send State", this.state);
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
  //   }
  constructor(props) {
    super();
    this.handleChange = this.handleChange.bind(this);
    this.handleAxis = this.handleAxis.bind(this);
  }

  handleChange(buttonName, pressed) {
    let change = {};
    change[buttonName] = pressed;
    this.setState(change);
    let temp_ang_z = 0;
    if (this.state.DPadLeft) {
      temp_ang_z = 1;
    } else if (this.state.DPadRight) {
      temp_ang_z = -1;
    }
    this.setState({
      vect: {
        lin_x: this.state.LeftStickX,
        lin_y: this.state.LeftStickY,
        lin_z: this.state.RightStickY,
        ang_x: 0,
        ang_y: 0,
        ang_z: temp_ang_z,
      },
    });

    //socket.emit("Send State", this.state.vect);
  }

  handleAxis(axisName, value, presiousValue) {
    // let change = {};
    // change[axisName] = value;
    // this.setState(change)

    // solve stick problem, code below is pseudo code
    // AxisStyleLeft = {relevant styles}
    // AxisStyleTop = {relevant styles}
    // Styles={ ...AxisStyleLeft, ...AxisStyleTop}

    switch (axisName) {
      case "LeftStickX":
        this.setState({
          LeftStickX: value,
          stick: { left: 98 + value * 20 + "px" },
        });
        // document.getElementById("stick").style.left = 98 + value * 20 + "px";
        break;
      case "RightStickX":
        this.setState({
          RightStickX: value,
          stick2: { left: 394 + value * 20 + "px" },
        });
        // document.getElementById("stick2").style.left = 394 + value * 20 + "px";
        break;
      case "LeftStickY":
        this.setState({
          LeftStickY: value,
          stick: { top: 175 + value * -20 + "px" },
        });
        // document.getElementById("stick").style.top = 175 + value * -20 + "px";
        break;
      case "RightStickY":
        this.setState({
          RightStickY: value,
          stick2: { top: 296 + value * -20 + "px" },
        });
        // document.getElementById("stick2").style.top = 296 + value * -20 + "px";
        break;
      case "LeftTrigger":
        this.setState({
          LeftTrigger: value,
        });
        break;
      case "RightTrigger":
        this.setState({
          RightTrigger: value,
        });
        break;
      default:
        console.log("Error: axisName not defined");
    }

    this.setState({
      vect: {
        lin_x: this.state.LeftStickY,
        lin_y: this.state.LeftStickX,
        lin_z: this.state.RightStickX,
        ang_x: 0,
        ang_y: 0,
        ang_z: 0,
      },
    });
    //socket.emit("Send State", this.state.vect);
  }

  componentDidUpdate() {
    console.log("Sending: ");
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
          <img src="/xboxImages/a.png" id="a" style={this.style.A} />
          <img src="/xboxImages/b.png" id="b" style={this.style.B} />
          <img src="/xboxImages/x.png" id="x" style={this.style.X} />
          <img src="/xboxImages/y.png" id="y" style={this.style.Y} />
          <img
            src="/xboxImages/left.png"
            id="left"
            style={this.style.DPadLeft}
          />
          <img
            src="/xboxImages/right.png"
            id="right"
            style={this.style.DPadRight}
          />
          <img src="/xboxImages/up.png" id="up" style={this.style.DPadUp} />
          <img
            src="/xboxImages/down.png"
            id="down"
            style={this.style.DPadDown}
          />
          <img
            src="/xboxImages/stick.png"
            id="stick"
            style={this.style.stick}
          />{" "}
          //!
          <img
            src="/xboxImages/stick2.png"
            id="stick2"
            style={this.style.stick2}
          />{" "}
          //!
          <img
            src="/xboxImages/bumperleft.png"
            id="bumpl"
            style={this.style.LB}
          />
          <img
            src="/xboxImages/bumperright.png"
            id="bumpr"
            style={this.style.RB}
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
          <img src="/xboxImages/left.png" id="back" style={this.style.Back} />
          <img
            src="/xboxImages/right.png"
            id="start"
            style={this.style.Start}
          />
        </div>
      </Gamepad>
    );
  }
}
