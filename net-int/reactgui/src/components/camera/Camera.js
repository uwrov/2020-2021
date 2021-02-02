import React from 'react';
import './Camera.css';

// This component is a visual representation of the movement vector
// that is being sent to the server by the controller.
export default class Camera extends React.Component {
   // Initialize the output vectors to zero.
   constructor(props) {
      super(props);
      this.socket = require('socket.io-client')('http://localhost:4040');
      this.state = {
         img_src: null
      };
      console.log("init camera")
      console.log(this.socket)
      this.socket.on("connected", () => {
         console.log("Connected")
      })

      this.socket.emit("Test")

      this.socket.on("Test", () => {
         console.log("test works")
      })

      this.socket.on("Image Display", (image) => {
         console.log("got image")
         this.setImage(image);
      });
      console.log("after socket")
   }

   setImage = (image) => {
      console.log("got image")
      let typed_array = new Uint8Array(image.image);
      const data = typed_array.reduce((acc, i) => acc += String.fromCharCode.apply(null, [i]), '');
      //const string_char = String.fromCharCode.apply(null, typed_array);
      let imageurl = "data:image/png;base64, " + data;
      this.setState({ img_src: imageurl });
   }
   click = () => {
     console.log("Click")
     console.log(this.socket)
     this.socket.emit("Test")
     console.log("Clicked")
     console.log(this.socket)
   }


   render() {
      return(
         <div>
            <button onClick={this.click}>Click Me</button>
            <img src={this.state.img_src} alt="Image Display"/>
         </div>
      );
   }
}
