import React from "react";

import { WIDGET_DICT } from "./WidgetLib.js";

import "./widgets.css";

let WINDOW_COUNT = [0, 0];

export class WidgetTree {
  constructor(updateCallback) {
    this.updateState = updateCallback;
    this.root = null;
  }
}

export class Branch {
  constructor() {
    this.WIN_ID = WINDOW_COUNT[isTest]++;
    this.children = [];
    this.visuals = {
      height: 100,
      width: 100
    }
  }
}

export class Leaf {
  constructor() {
    this.WIN_ID = WINDOW_COUNT[isTest]++;
    this.tabs = [];
    this.openTab = 0;
    this.visuals = {
      height: 100,
      width: 100
    }
  }
}

export class Widget {
  constructor(type) {
    this.type = type;
    this.savedProps = {
      saveProp: this.saveProp,
    };
  }

  saveProp = (object) => {
    this.savedProps = Object.assign(this.savedProps, object);
  };
}

/**
 *  Returns the DOM representation of the given widget leaf component.
 *  Component must be defined in the WidgetLib dictionary.
 *
 *  @param {Leaf} component a widget component to generate
 *
 *  @return {React.Component} React Component generated from the leaf.
 */
export function generateComponent(component) {
  if (component !== null && component !== undefined) {
    return WIDGET_DICT(component);
  }
}

export default {};
