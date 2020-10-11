import React from 'react';


export default function Node(props) {
   let name = "";
   if(props.id === "left") {
      name = "left";
   } else if (props.id === "right") {
      name = "right";
   } else if (props.id === "up") {
      name = "up";
   } else if (props.id === "down") {
      name = "down";
   }
   
   return (<div className="node" id={props.display} >{name}</div>)
}