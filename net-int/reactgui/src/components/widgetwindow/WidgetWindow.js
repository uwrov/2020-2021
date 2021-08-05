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
    rowHeight: 100,
    columns: 8,
    layout: [
      {w: 6, h: 6, x: 0, y: 0, i: "key0"},
      {w: 2, h: 3, x: 6, y: 0, i: "key1"},
      {w: 2, h: 3, x: 6, y: 3, i: "key2"},
    ]
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
          rowHeight={this.state.rowHeight} width={this.state.width}
          layout={this.state.layout}
          onLayoutChange={this.handleLayoutChange}>
        {this.renderWidgets()}
      </GridLayout>
    )
  }

  renderWidgets() {
    if('widgets' in this.props) {
      return this.props.widgets.map((widget, i) => {
        return (
          <div key={"key" + i}>
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

  handleLayoutChange = (layout) => {
    console.log(layout);
    this.setState({layout: layout});
  }

  getLayout() {
    return this.state.layout
  }
}

export default WidgetWindow;
