import React from "react";
import Tab from "./Tab";

class Audio extends Tab {
  constructor(props) {
    super(props);
  }

  state = {
    isOpen: false,
  };

  renderSettings() {
    return <p>Audio</p>;
  }
}

export default Audio;
