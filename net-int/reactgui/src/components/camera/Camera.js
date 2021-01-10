
import React from 'react';
import './Camera.css';

const socket = require('socket.io-client')('http://localhost:4040');

// This component is a visual representation of the movement vector
// that is being sent to the server by the controller.
export default class Controller extends React.Component {
   // Initialize the output vectors to zero.
   constructor(props) {
      super();
      this.state = {
         img_src: null
      };

      socket.on("Image Display", (image) => {
         this.setImage(image);
      });
   }

   setImage = (image) => {
      let typed_array = new Uint8Array(image.image);
      const data = typed_array.reduce((acc, i) => acc += String.fromCharCode.apply(null, [i]), '');
      //const string_char = String.fromCharCode.apply(null, typed_array);
      let imageurl = "data:image/png;base64, " + data;
      this.setState({ image_src: imageurl });
   }


   render() {
      return(
         <div>
            <img src={this.state.image_src} alt="Image Display"/>
         </div>
      );
   }
}
