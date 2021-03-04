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
        className={`set ${this.state.isOpen ? this.state.isOpen : ""}`}
        onClick={() => {
          this.onClickFunc(this);
          this.handleTabClick();
        }}
      >
        {this.constructor.name}
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
