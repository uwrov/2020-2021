import React from "react";
import "./RosCamera.css";

export default class Camera extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require("socket.io-client")("http://localhost:4040");
    this.state = {
      channels: [{id : "id_1", image : "https://cdn.mos.cms.futurecdn.net/42E9as7NaTaAi4A6JcuFwG-1200-80.jpg"}, {id : "id_2", image : "https://www.thermofisher.com/blog/food/wp-content/uploads/sites/5/2015/08/single_strawberry__isolated_on_a_white_background.jpg"}],
      curr: 0
    };

    console.log(this.socket);

    // make a getid socket

    this.socket.on("Image Display", (image) => {
      console.log("got image");
      var seen = new Boolean(false);
      for (var i = 0; i < this.state.channels.length; i++) {
          if (this.state.channels[i].id === image.id){
            this.state.channels[i] = {id : image.id, image : image.image};
            seen = Boolean(true);
            break;
          }
      }
      if (!seen) {
        this.state.channels.push({id : image.id, image : image.image});
      }
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
    return this.state.channels.map((item, i) => {
      return(<option value={i}>{item.id}</option>);
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
        <img src={this.state.channels[this.state.curr].image} alt="Image Display" className="image" />
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
