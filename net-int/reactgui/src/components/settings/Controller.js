import React from "react";
import Tab from "./Tab";

class Controller extends Tab {
  constructor(props) {
    super(props);
  }

  state = {
    isOpen: false,
  };

  renderSettings() {
    return <p>CONTROLLER</p>;
  }
}

export default Controller;
