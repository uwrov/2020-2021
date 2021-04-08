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
      img_src_3:
        "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg"
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
    if (image.id == "front_cam") {
      this.setState({ img_src_1: imageurl })
    } else if (image.id == "down_cam") {
      this.setState({ img_src_2: imageurl })
    } else if (image.id == "img_sub") {
      this.setState({ img_src_3: imageurl })
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
        <img src={this.state.img_src_3} alt="Image Display" className="image_3" />
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
