import React from "react";

class Tab {
  constructor(func) {
    this.onClickFunc = func;
  }

  state = {
    isOpen: false,
  };

  renderOpenWindow() {
    return <div className="generalSettings">{this.renderSettings()}</div>;
  }

  renderSettings() {}

  renderClosed() {
    return (
      <div
        className={`single-setting ${
          this.state.isOpen ? this.state.isOpen : ""
        }`}
        onClick={() => {
          this.onClickFunc(this);
          this.handleTabClick();
        }}
      >
        <div className="options-wrapper">
          <img
            className={"options-icons"}
            src={"settingsImages/" + this.constructor.name + ".png"}
          ></img>
          <p className="options-names">{this.constructor.name}</p>
        </div>
      </div>
    );
  }

  handleTabClick = () => {
    this.state.isOpen = !this.state.isOpen;
  };

  tabState() {
    return this.state.isOpen;
  }
}

export default Tab;
