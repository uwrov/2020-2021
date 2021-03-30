import React from "react";

// This component represents the nodes that light up when
// a particular key is pressed.
export default function node(props) {
  let name = "";
  // Assigns a text to the Node. Will be empty if not one of the
  // predefined keys.
  if (props.id === "left") {
    name = "left";
  } else if (props.id === "right") {
    name = "right";
  } else if (props.id === "front") {
    name = "front";
  } else if (props.id === "back") {
    name = "back";
  } else if (props.id === "up") {
    name = "up";
  } else if (props.id === "down") {
    name = "down";
  }

  return (
    <div className="node" id={props.display}>
      {name}
    </div>
  );
}
