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
      <GridLayout className="layout" cols={this.state.columns}
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
            {getWidgetComponent(widget)}
          </div>
        );
      });
    }
  }
}

export default WidgetWindow;
