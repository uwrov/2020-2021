import React from "react";
import icon from "./sicon.png";
import GeneralSettings from "./GeneralSettings";
import Controller from "./Controller";
import KeyBindings from "./KeyBindings";
import "./settings.css";

class Settings extends React.Component {
  state = {
    openTab: 0,
    tabs: [],
  };

  constructor(props) {
    super(props);
    let generalSettings = new GeneralSettings(this.changeTab);
    let controller = new Controller(this.changeTab);
    let keys = new KeyBindings(this.changeTab);
    this.state.tabs.push(generalSettings);
    this.state.tabs.push(controller);
    this.state.tabs.push(keys);
  }

  render() {
    return (
      <div id="fullpage">
        <img src={icon} onClick={this.props.onClick} />
        <div className={this.props.active}>
          <div className="window">
            <div id="exit-button" onClick={this.props.onClick}></div>
            <h1 id="settings-title">Settings</h1>
          </div>
          <div className="sidebar">{this.renderClosedTabs()}</div>
          {this.state.tabs[this.state.openTab].renderOpenWindow()}
        </div>
      </div>
    );
  }

  renderClosedTabs() {
    return (
      <div id="test">
        {this.state.tabs.map((element) => element.renderClosed())}
      </div>
    );
  }

  changeTab = (tab) => {
    let chosenTab = tab;
    this.setState({ openTab: this.state.tabs.indexOf(tab) });

    for (let i = 0; i < this.state.tabs.length; i++) {
      if (this.state.tabs[i] != chosenTab) {
        this.state.tabs[i].state.isOpen = false;
      }
    }
  };

  displaySidebar = () => {
    const numbers = this.state.settings;
    const list = numbers.map((number) => (
      <div key={number.toString()} id={number.toString()} className="set">
        {number.toString()}
      </div>
    ));

    return <div>{list}</div>;
  };
}

export default Settings;
