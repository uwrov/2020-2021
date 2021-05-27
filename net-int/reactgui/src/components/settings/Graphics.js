import React from "react";
import Tab from "./Tab";

class Graphics extends Tab {
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
    return <p>Graphics</p>;
  }
}

export default Graphics;
