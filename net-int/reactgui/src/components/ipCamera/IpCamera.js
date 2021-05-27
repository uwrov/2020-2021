import React from "react";
import "./IpCamera.css";

export default class Camera extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require("socket.io-client")("http://localhost:4040");
    this.state = {
      img_src:
        "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg",
      ip_add_pre: null,
      ip_add_post: null,
      hide: false,
    };

    console.log(this.socket);
    this.socket.on("Image Display", (image) => {
      console.log("got image");
      this.setImage(image);
    });
  }

  setImage = (image) => {
    let typed_array = new Uint8Array(image.image);
    const data = typed_array.reduce(
      (acc, i) => (acc += String.fromCharCode.apply(null, [i])),
      ""
    );
    //const string_char = String.fromCharCode.apply(null, typed_array);
    let imageurl = "data:image/png;base64, " + data;
    this.setState({ img_src: imageurl });
  };

  render() {
    return (
      <div className="camera">
        <div className="buttons-wrapper">
          <label className="switch">
            <input type="checkbox"></input>
            <span
              class="slider round"
              onClick={() => {
                let curr = this.state.hide;
                this.setState({
                  hide: !curr,
                });
              }}
            ></span>
          </label>
          <div className="camera-buttons">
            {!this.state.hide ? (
              <input
                type="text"
                className="ip-textbox"
                placeholder="  IP-Address..."
                onChange={(e) => {
                  this.setState({
                    ip_add_pre: e.target.value,
                  });
                }}
              />
            ) : null}
            {!this.state.hide ? (
              <div
                className="submit-button"
                onClick={() => {
                  this.setState({
                    ip_add_post: this.state.ip_add_pre,
                  });
                }}
              >
                {" "}
                Submit{" "}
              </div>
            ) : null}
          </div>
        </div>
        <img
          src={this.state.ip_add_post}
          alt="IP Camera"
          className="ip-camera"
        />
      </div>
    );
  }
}
