
import React from 'react';
import Key from './key';
import Setting from './setting';
import './configuration.css';

export default class configuration extends React.Component {

   constructor() {
      super();
      this.state = {
         config: {
            front: 87,
            back: 83,
            left: 65,
            right: 68,
            up: 84,
            down: 71
         },
         change: false
      };
   }

   render() {
      return(
         <div className="config">
            <Key config={this.state.config}/>
            <Setting config={this.state.config} clickChange={this.toggleChange} change={this.state.change}/>
         </div>
      );
   }

   toggleChange = () => {
      let isChange = !this.state.change;
      this.setState({
         change: isChange
      });
   }
}
