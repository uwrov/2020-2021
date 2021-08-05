import React from "react";
import "./RosCamera.css";

const ERROR_IMG =
  "https://developers.google.com/maps/documentation/streetview/images/error-image-generic.png";

export default class Camera extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require("socket.io-client")("http://localhost:4040");
    this.state = {
      ids: ["id_1", "id_2", "id_3"],
      channels: {
        id_1:
          "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg",
        id_2:
          "https://www.thermofisher.com/blog/food/wp-content/uploads/sites/5/2015/08/single_strawberry__isolated_on_a_white_background.jpg",
        id_3: "http://uwrov.org/images/Nautilus/Nau8.JPG",
      },
      curr: 0,
      hide: false,
      currImage: null,
      rotation: 0
    };

    this.socket.on("Image Display", this.updateImage);
    this.socket.on("IDs", this.updateIDS);
    this.getIDs();
  }

  getIDs = () => {
    this.socket.emit("Get IDs");
    console.log("GETIDS");
  };

  updateIDS = (data) => {
    console.log("---------------GETTING IDS-------------------");
    console.log(data.ids);
    this.setState({ ids: data.ids });
    let newChannels = {};
    data.ids.forEach((item, i) => {
      newChannels[item] = ERROR_IMG;
    });
    this.setState({ channels: newChannels });
  };

  updateImage = (image) => {
    console.log("got image");
    let channelsCopy = Object.assign({}, this.state.channels);
    if (true) {
      //channelsCopy[image.id] !== undefined) {
      channelsCopy[image.id] = decodeImageToURL(image.image);
      this.setState({ channels: channelsCopy });
    }
    //this.setState({currImage: decodeImageToURL(image.image)});
  };

  renderOptions() {
    return this.state.ids.map((item, i) => {
      return <option value={i}>{item}</option>;
    });
  }

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
          <div className="ros-camera-buttons">
            {!this.state.hide ? (
              <span className="channel-label">Channel:</span>
            ) : null}
            {!this.state.hide ? (
              <select
                className="channels"
                onChange={(event) => {
                  this.setState({ curr: event.target.value });
                }}
              >
                {this.renderOptions()}
              </select>
            ) : null}
          </div>
        </div>
        <button className="rotate-button" onClick={this.rotate}>rotate</button>
        <img
          src={this.getCurrentImageId()}
          alt="Image Display"
          style={this.generateRotationStyle()}
          className="image"
        />
      </div>
    );
  }

  getCurrentImageId() {
    //return this.state.currImage;
    return this.state.channels[this.state.ids[this.state.curr]];
  }

  generateRotationStyle = () => {
    return (
      {
        'transform': 'rotate(' + (90 * this.state.rotation) + 'deg)',
        '-webkit-transform': 'rotate(' + (90 * this.state.rotation) + 'deg)'
      }
    );
  }

  rotate = () => {
    let next = this.state.rotation + 1;
    if(next > 3) next = 0;
    this.setState({rotation: next});
  }
}

let decodeImageToURL = (image) => {
  let typed_array = new Uint8Array(image);
  //const data = typed_array.reduce((acc, i) => acc += String.fromCharCode.apply(null, [i]), '');
  const data = btoa(String.fromCharCode.apply(null, typed_array));
  let imageurl = "data:image/png;base64, " + data;
  console.log(imageurl);
  return imageurl;
};
