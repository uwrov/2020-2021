import React from 'react';
import Graphics from './Graphics';
import Audio from './Audio';
import KeyBindings from './KeyBindings';
import './settings.css'
import 'bootstrap/dist/css/bootstrap.min.css';


class Settings extends React.Component {



  state = {
    openTab: 0,
    tabs: [],
  }

  constructor(props) {
    super(props);
    let graphics = new Graphics(this.changeTab);
    let audio = new Audio(this.changeTab);
    let keys = new KeyBindings(this.changeTab);
    this.state.tabs.push(graphics);
    this.state.tabs.push(audio);
    this.state.tabs.push(keys);
  }


  render() {
    return (
        <div className="settings-widget">
          <div className="sidebar">
          <div className="window">
            <img id="settings-icon" src="settingsImages/sicon.png"></img>
            <p id="settings-title">Settings</p>
          </div>
            {this.renderClosedTabs()}
          </div>
          {this.state.tabs[this.state.openTab].renderOpenWindow()}
        </div>
    )
  }

  renderClosedTabs() {
    return (
      <div id="options">
        {
          this.state.tabs.map(
            element => element.renderClosed()
          )
        }
      </div>
    )
  }

  changeTab = (tab) => {
    let chosenTab = tab;
    this.setState({openTab: this.state.tabs.indexOf(tab)});

    for (let i = 0; i < this.state.tabs.length; i++) {
      if (this.state.tabs[i] != chosenTab) {
        this.state.tabs[i].state.isOpen = false;
      }
    }
  }


  displaySidebar = () => {
    const numbers = this.state.settings;
    const list = numbers.map((number) =>
      <div key={number.toString()} id={number.toString()} className="single-setting">{number.toString()}
      </div>
    );

    return (
    <div>{list}</div>
    );
  }
}

export default Settings;
