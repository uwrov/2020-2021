import React, { Component } from "react";
import "./Console.css";

//const socket = require("socket.io-client")("http://localhost:4046");

class Console extends Component {
  static thisConsole;

  constructor(props) {
    super(props);

    this.prevArgs = []; // list of prev args
    this.argCount = -1;
    this.tempArgNum = 0;
    //this.resize = "minimize";
    this.state = { resize: "maximize", text: "", consoleWindow: "", rows: 1 };

    //this.showConsole(true)
    this.consoleStorage = window.localStorage;
    this.data = "Console created.\nListening...\n";
    var temp = "Console created.\nListening...\n";
    this.consoleStorage.setItem("ConsoleData", temp); //set empty console
    this.socket = require("socket.io-client")("http://localhost:4040");
    // listen for server logs
    this.setSocketOn();
  }

  setSocketOn = () => {
    this.socket.on("Print Console Logs", (data) => {
      let temp = this.consoleStorage.getItem("ConsoleData");
      data.forEach((el) => {
        temp += "$>Type: " + el.type + "Message: " + el.message + "\n";
        this.consoleStorage.removeItem("ConsoleData");
        this.consoleStorage.setItem("ConsoleData", temp);
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        });
      });
    });
  };

  addArgs = () => {
    if (this.state.text !== undefined && this.state.text !== "\\") {
      this.prevArgs.push(this.state.text);
      this.argCount += 1; // increment counter
      //console.log(this.argCount);
    }
  };

  getPrevArg = () => {
    if (this.tempArgNum >= 0) {
      this.setState({
        text: this.prevArgs[this.tempArgNum],
      });
      this.tempArgNum -= 1;
    } else {
      this.setState({
        text: this.prevArgs[0],
      });
    }
  };

  updateConsole = () => {
    try {
      if (this.data === this.consoleStorage.getItem("ConsoleData")) {
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        }); //update the console every 100 ms
      } else {
        this.data = this.consoleStorage.getItem("ConsoleData");
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        });
        var textarea = document.getElementById("outputText");
        textarea.scrollTop = textarea.scrollHeight;
      }
    } catch (e) {
      console.log(e);
    }
  };

  handleBackSlash = () => {
    console.log("triggred");
    this.getPrevArg();
  };

  handleEnter = () => {
    this.addArgs();
    this.tempArgNum = this.argCount; // reset current prev arg to start again
    var temp = this.consoleStorage.getItem("ConsoleData");
    var command = this.state.text + " $";
    var commandArr = Array.from(command);
    if (commandArr[0] === "/") {
      //command handle
      var commandInfo = command.split(" ");
      if (commandInfo[0] === "/run") {
        if (commandInfo[1] === "$" || commandInfo[1] === "") {
          // no arg specified
          temp += "$>" + this.state.text + "\n";
          temp += "$>No file specified. Usage '/run filename'\n";
        } else {
          // args are present
          temp += "$>" + this.state.text + "\n";
          let data = '{ "command" : "run", "arg1" : "' + commandInfo[1] + '" }'; // create data
          console.log(data); // debug
          this.socket.emit("Send Commands", data); // send data to server
        }
        this.consoleStorage.removeItem("ConsoleData");
        this.consoleStorage.setItem("ConsoleData", temp);
        this.setState({
          text: "", // clear the input field
        });
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        });
      } else if (commandInfo[0] === "/list") {
        // args are present
        temp += "$>" + this.state.text + "\n";
        let data = '{ "command" : "list" }'; // create data
        console.log("oof: " + data); // debug
        this.socket.emit("Send Commands", data); // send data to server
        this.consoleStorage.removeItem("ConsoleData");
        this.consoleStorage.setItem("ConsoleData", temp);
        this.setState({
          text: "", // clear the input field
        });
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        });
      } else {
        temp += "$>command not recognized...\n";
        this.consoleStorage.removeItem("ConsoleData");
        this.consoleStorage.setItem("ConsoleData", temp);
        this.setState({
          text: "", // clear the input field
        });
        this.setState({
          consoleWindow: this.consoleStorage.getItem("ConsoleData"),
        });
      }
    } else {
      console.log(temp);
      if (this.state.text === undefined) {
        temp += "$>\n";
      } else {
        temp += "$>" + this.state.text + "\n";
      }
      console.log(temp);
      this.consoleStorage.removeItem("ConsoleData");
      this.consoleStorage.setItem("ConsoleData", temp);
      this.setState({
        text: "", // clear the input field
      });
      this.setState({
        consoleWindow: this.consoleStorage.getItem("ConsoleData"),
      });
    }
  };

  handleChange = (event) => {
    this.setState({ text: event.target.value });
  };

  keyPressed = (event) => {
    console.log(event.key);
    if (event.key === "Enter") {
      //if enter is pressed
      event.preventDefault();
      this.handleEnter();
    } else if (event.key === "\\") {
      event.preventDefault();
      this.handleBackSlash();
    }
  };

  handleButtonClick = () => {
    if (this.state.resize === "minimize") {
      this.setState({
        resize: "maximize",
        rows: 1,
      });
    } else {
      this.setState({
        resize: "minimize",
        rows: 20,
      });
    }
  };

  render() {
    if (this.props.show === true) {
      return (
        <div id="console">
          <div>
            <button onClick={this.handleButtonClick}>
              {this.state.resize}
            </button>
          </div>
          <div>
            <textarea
              id="outputText"
              value={this.state.consoleWindow}
              disabled
              onChange={this.handleChange.bind(this)}
              onKeyPress={this.keyPressed.bind(this)}
              rows={this.state.rows}
              cols="106"
            />
          </div>
          <div>
            <textarea
              id="in"
              rows="1"
              cols="106"
              name="t"
              value={this.state.text}
              placeholder="your command here"
              type="text"
              onChange={this.handleChange.bind(this)}
              onKeyPress={this.keyPressed.bind(this)}
            />
          </div>
        </div>
      );
    } else {
      return <div></div>;
    }
  }
}

export default Console;
