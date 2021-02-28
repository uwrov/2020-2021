import React from "react";
import Tab from "./Tab";

class GeneralSettings extends Tab {
  constructor(props) {
    super(props);
  }

  state = {
    isOpen: false,
    settings: {
      id: 0,
      color: "blue",
    },
  };

  renderSettings() {
    return <p>General Settings</p>;
  }
}

export default GeneralSettings;
