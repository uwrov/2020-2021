import React, {Component} from 'react';

/**
 * A text field that allows the user to enter the list of edges.
 * Also contains the buttons that the user will use to interact with the app.
 */
class ScriptRunner extends Component {
    socket;
    constructor(props) {
        super(props)
        this.state = {
            command: "None", // Starts with no start building
        };
        this.socket = require("socket.io-client")("http://localhost:4040");
    }
    //
    // Changes the currently selected start building
    handleScriptChange= (event) => {
        this.setState({script: event.target.value});
    }
    //
    // // Changes the currently destination start building
    // handleDestChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    //     this.setState({dest: event.target.value});
    //     this.props.changeStartOrDest(event.target.value, false);
    // }

    // Triggers the app to calculate and draw a new path if the start and destination are valid
    handleRunScript = () => {
        if(this.state.script === "None") {
            alert("Please choose a script");
        }else{
            this.socket.emit("Send Commands", this.state.script)
        }
        this.socket.on("Print Console Logs", (logs)=> {
            for (let log in logs){

            }
        });
    }

    // Resets the app to the default state
    handleClear= () => {
        this.setState({
            script: "None",
        });
    }

    render() {
        return (
            <div id = "script-selector">
                <br/>Script:
                <select value={this.state.script} onChange={this.handleScriptChange}>
                    <option value="None">Select a Script</option>
                </select>
                {/*Destination:*/}
                {/*<select value={this.state.dest} onChange={this.handleDestChange}>*/}
                {/*    <option value="None">Select a Destination location</option>*/}
                {/*    {this.props.options}*/}
                {/*</select> <br/>*/}
                <button onClick={this.handleRunScript}>Run Script</button>
                <button onClick={this.handleClear}>Clear</button>
                <button onClick={this.sendSignal}>Send Activate Signal</button>
            </div>
        );
    }

    sendSignal = () => {
      this.socket.emit("Activate Script");
    }
}

export default ScriptRunner;
