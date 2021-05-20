import React from "react";
import Tab from "./Tab";

class KeyBindings extends Tab {
  constructor(props) {
    super(props);
  }

  state = {
    isOpen: false,
  };

  renderSettings() {
    return <p>Key Bindings</p>;
  }
}

export default KeyBindings;
