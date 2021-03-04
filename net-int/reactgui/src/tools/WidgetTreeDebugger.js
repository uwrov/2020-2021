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
    ctx.fillStyle = "#AAAAFF";
    ctx.fillRect(0, 0, 150, 75);
    this.drawTreeLevel(this.props.tree, 0);
  }

  render() {
    let rend = (
      <div className="widget-tree-debugger">
        <canvas ref={(mount) => {this.mount = mount}}>
        </canvas>
      </div>
    );
    return rend;
  }

  drawTreeLevel(object, level, x=0) {
    let y = level * HEIGHT_SPACE;
    let ctx = this.mount.getContext("2d");
    if(object instanceof WT.Leaf) {
      ctx.fillStyle = "#4455AA";
      ctx.fillRect(x * WIDTH_SPACE, y, 50, 50);
    } else if(object != undefined || object != null) {
      ctx.fillStyle = "#8855AA";
      ctx.fillRect(x * WIDTH_SPACE, y, 50, 50)
      object.child.forEach((child, i) => {
        this.drawTreeLevel(child, level + 1, x);
        x += 1;
      });
    }
  }
}

export default WidgetTreeDebugger;
