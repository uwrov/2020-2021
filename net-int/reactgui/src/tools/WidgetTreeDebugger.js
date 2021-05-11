import React from "react";

import WT from "../datastructs/WidgetTree.js";

import "./WidgetTreeDebugger.css";

const HEIGHT_SPACE = 70;
const WIDTH_SPACE = 70;
const NODE_HEIGHT = 50;
const NODE_WIDTH = 50;

class WidgetTreeDebugger extends React.Component {


  componentDidMount() {
    var ctx = this.mount.getContext("2d");
    ctx.font = "30px Arial";
    ctx.fillStyle = "#AAAAFF";
    ctx.fillRect(0, 0, 1000, 500);
    ctx.fillStyle = "#000000";
    ctx.fillText("Widget Tree View", 10, 35);
    this.drawTreeLevel(this.props.tree, 0.5, 4);
  }

  render() {
    let rend = (
      <div className="widget-tree-debugger">
        <canvas width="1000px" height= "700px" ref={(mount) => {this.mount = mount}}>
        </canvas>
      </div>
    );
    return rend;
  }

  drawTreeLevel(object, level, x=0) {
    let y = level * HEIGHT_SPACE;
    let ctx = this.mount.getContext("2d");
    let sX = x;
    if(object instanceof WT.Leaf) {
      ctx.fillStyle = "#4455AA";
      ctx.fillRect(x * WIDTH_SPACE, y, 50, 50);
    } else if(object != undefined || object != null) {
      ctx.fillStyle = "#8855AA";
      ctx.fillRect(x * WIDTH_SPACE, y, 50, 50)
      if(object.hasLeafChildren) {
        ctx.fillStyle = "#4455AA";
        ctx.fillRect(x * WIDTH_SPACE, y + HEIGHT_SPACE, 50, 50);
        ctx.fillStyle = "#000000";
        ctx.fillText(object.child.length, x * WIDTH_SPACE + 15, y + HEIGHT_SPACE + 34);
        ctx.fillStyle = "#000000";
        ctx.beginPath();
        ctx.moveTo(x * WIDTH_SPACE + 25, y + HEIGHT_SPACE);
        ctx.lineTo(sX * WIDTH_SPACE + 25, y - 30 + HEIGHT_SPACE);
        ctx.stroke();
      } else {
        object.child.forEach((child, i) => {
          this.drawTreeLevel(child, level + 1, x);
          ctx.fillStyle = "#000000";
          ctx.beginPath();
          ctx.moveTo(x * WIDTH_SPACE + 25, y + HEIGHT_SPACE);
          ctx.lineTo(sX * WIDTH_SPACE + 25, y - 30 + HEIGHT_SPACE);
          ctx.stroke();
          x += 1;
        });
      }
    }
  }
}

export default WidgetTreeDebugger;
