import React from "react";
import "./RosCamera.css";

export default class Camera extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require("socket.io-client")("http://localhost:4040");
    this.state = {
      channels: ["https://i2.wp.com/ceklog.kindel.com/wp-content/uploads/2013/02/firefox_2018-07-10_07-50-11.png?fit=641%2C618&ssl=1", "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg", "https://www.thermofisher.com/blog/food/wp-content/uploads/sites/5/2015/08/single_strawberry__isolated_on_a_white_background.jpg", "https://images-na.ssl-images-amazon.com/images/I/71%2BqAJehpkL._SL1500_.jpg"],
      curr: 0
    };

    console.log(this.socket);

    // make a getid socket

    this.socket.on("Image Display", (image) => {
      console.log("got image");
      console.log(image.id);
      this.setImage(image);
    });
  }

  setImage = (image) => {
    var d = new Date();
    var n = d.getTime();
    let typed_array = new Uint8Array(image.image);
    const data = typed_array.reduce(
      (acc, i) => (acc += String.fromCharCode.apply(null, [i])),
      ""
    );
    //const string_char = String.fromCharCode.apply(null, typed_array);
    let imageurl = "data:image/png;base64, " + data;
    let channels = this.state.channels;
    channels[image.id] = imageurl;
    this.setState({ channels: channels});
    var d2 = new Date();
    var m = d2.getTime();
    console.log(m-n)
  };

  renderOptions() {
    return this.state.channels.map(function(url, i) {
      return(<option value={i}>{i + 1}</option>);
    });
  }

  render() {
    return (
      <div className="camera">
        <p>Channel: </p>
        <select id="channels" onChange={(event) => {
          this.setState({
            curr: event.target.value
          });
        }}>
          {this.renderOptions()}
        </select>
        <img src={this.state.channels[this.state.curr]} alt="Image Display" className="image" />
      </div>
    );
  }

  
  // render() {
  //   return (
  //     <div className="camera">
  //       <img src={this.state.img_src} alt="Image Display" className="image" />
  //     </div>
  //   );
  // }
}
