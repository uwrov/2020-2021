
import React from 'react';
import './setting.css';

export default function setting(props) {
   return(
      <div className="setting">
         <span className={"dot"} id={props.change ? "on" : "off"} onClick={props.clickChange}> </span>
      </div>
   );
}
