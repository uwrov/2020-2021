import React from "react";
import GridLayout from 'react-grid-layout';
import Camera from '../ipCamera/IpCamera.js';
import { getWidgetComponent } from '../../datastructs/WidgetLib.js';

import "./WidgetWindow.css";
import 'react-grid-layout/css/styles.css'

export class WidgetWindow extends React.Component {
  state = {
    editMode: false,
    width: 0,
    rowHeight: 60,
    columns: 12,
  }

  constructor(props) {
    super(props);
  }

  updateDimensions = () => {
    let w;
    if('width' in this.props) {
      w = this.props.width * window.innerWidth;
    } else {
      w = window.innerWidth;
    }
    this.setState({width: w});
  };

  componentDidMount() {
    this.updateDimensions();
    window.addEventListener('resize', this.updateDimensions);
  }

  componentWillUnmount() {
    window.removeEventListener('resize', this.updateDimensions);
  }

  render() {
    return (
      <GridLayout className="layout" cols={this.state.columns} draggableHandle=".drag-handle"
          rowHeight={this.state.rowHeight} width={this.state.width}>
        {this.renderWidgets()}
      </GridLayout>
    )
  }

  renderWidgets() {
    let count = 0;
    if('widgets' in this.props) {
      return this.props.widgets.map((widget) => {
        count++;
        return (
          <div key={"key" + count} data-grid={{x: count, y: 0, w: 5, h: 5, isResizable: true}}>
            <div className="widget-tab-header drag-handle">
              <a>{widget.type}</a>
              <span
                className="tab-exit-button"
                onClick={() => {
                  let newWidgets = this.props.widgets.slice(0);
                  let i = newWidgets.indexOf(widget);
                  if (i !== -1) {
                    newWidgets.splice(i, 1);
                  }
                  this.props.update(newWidgets);
                  console.log("click!!!!" + i);
                }}
              >
                &times;
              </span>
            </div>
            <div className="widget-content">
              {getWidgetComponent(widget)}
            </div>
          </div>
        );
      });
    }
  }
}

export default WidgetWindow;
