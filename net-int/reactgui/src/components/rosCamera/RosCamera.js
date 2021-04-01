import React from "react";
import "./RosCamera.css";

export default class Camera extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require("socket.io-client")("http://localhost:4040");
    this.state = {
      img_src_1:
        "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg",
      img_src_2:
        "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg",
    };

    console.log(this.socket);
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
    if (image.id == 1) {
      this.setState({ img_src_1: imageurl })
    } else if (image.id == 0) {
      this.setState({ img_src_2: imageurl })
    }
    // this.setState({ img_src: imageurl });
    var d2 = new Date();
    var m = d2.getTime();
    console.log(m-n)
  };

  render() {
    return (
      <div className="camera1">
        <img src={this.state.img_src_1} alt="Image Display" className="image_1" />
        <img src={this.state.img_src_2} alt="Image Display" className="image_2" />
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
