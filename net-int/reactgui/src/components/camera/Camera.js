import React from 'react';
import './Camera.css';

export default class Camera extends React.Component {
   constructor(props) {
      super(props);
      this.socket = require('socket.io-client')('http://localhost:4040');
      this.state = {
         img_src: null
      };

      console.log(this.socket)
      this.socket.on("Image Display", (image) => {
         console.log("got image")
         this.setImage(image);
      });
   }

   setImage = (image) => {
      let typed_array = new Uint8Array(image.image);
      const data = typed_array.reduce((acc, i) => acc += String.fromCharCode.apply(null, [i]), '');
      //const string_char = String.fromCharCode.apply(null, typed_array);
      let imageurl = "data:image/png;base64, " + data;
      this.setState({ img_src: imageurl });
   }

   render() {
      return(
         <div>
            <img src={this.state.img_src} alt="Image Display"/>
         </div>
      );
   }
}
