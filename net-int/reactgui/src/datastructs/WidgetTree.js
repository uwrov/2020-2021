import React from "react";

import { WIDGET_DICT } from "./WidgetLib.js";

import "./widgets.css";

let WINDOW_COUNT = [0, 0];

export class Window {
  constructor(isTest = 0) {
    this.WIN_ID = WINDOW_COUNT[isTest]++;
    this.hasLeafChildren = false;
    this.child = [];
    this.openTab = 0;
    this.style = {};
  }
}

export class Leaf {
  constructor(type) {
    this.type = type;
  }
}

// object: Object to be added to. Must be a window.
// node: Node to be added. Must be a window or a leaf
// If adding a window to a parent window, adds the window to the parent window (Case 2). If the
// parent window had leaves, adds a new window between the parent window and the leaves (Case 1)
// If adding a leaf to a window, adds the leaf to the window if it has other leafs (Case 3)
// Otherwise, finds the leftmost window with leaves and adds the leaf to that window (Case 4)
export function add(object, node,isTest = 0) {
   if (object instanceof Window){
      if(node instanceof Window){
         // Case 1
         if(object.hasLeafChildren){
            let newWindow = new Window(isTest);
            newWindow.hasLeafChildren = true;
            for (let i = 0; i < object.child.length; i++) {
               add(newWindow, object.child[i]);
            }
            object.child = [];
            object.child.push(newWindow);
            object.hasLeafChildren = false;
         }
         // Case 2
         object.child.push(node);
      } else if (node instanceof Leaf){
         // Case 3
         if(object.hasLeafChildren){
            object.child.push(node);
         } else{
            // Case 4
            object.child[0]= add(object.child[0], node);
         }
      } else {
        throw new Error("The second param must be of type Window or Leaf");
      }
  } else {
    throw new Error("The first param must be of type Window");
  }
  return object;
}

// Object must be a Window
// TODO comment all cases.
export function remove(object, windowID, componentID, isRoot = true) {
  if (object instanceof Window) {
    if (object.hasLeafChildren) {
      if (object.windowID === windowID) {
        object.child.splice(componentID, 1);
      }
      if (object.child.length=== 0 && !isRoot){
        return null;
      }
    } else {
      //Call remove on all this window's children windows
      for (let i = 0; i < object.child.length; i++) {
        let removed = remove(object.child[i], windowID, componentID,false);
        // Delete a child if it becomes null
        if (removed === null){
          object.child.splice(0,1);
        } else{
          object.child[i] = removed;
        }
        // Remove the intermediary window if a window has only 1 window child
        if (object.child.length=== 1){
          return object.child[0];
        }
      }
    }
  } else {
    throw "object must be a Window";
  }
  return object;
}

/**
 *  Returns either a leaf or window from the given WindowID.
 *
 *
 */
export function get(object, windowId, componentId) {
  if (object instanceof Window) {
    if (!object.hasLeafChildren) {
      for (let i = 0; i < object.child.length; i++) {
        let obj = get(object.child[i], windowId, componentId);
        if (obj !== null) {
          return obj;
        }
      }
      return null;
    } else {
      if (object.WIN_ID === windowId && object.child.length > componentId) {
        if (componentId < 0) return object;
        else if (object.child.length > componentId)
          return object.child[componentId];
      }
      return null;
    }
  } else {
    return null;
  }
}

export function setTab(object, windowId, tabId) {
  let window = get(object, windowId, -1);
  if (window !== null && tabId < window.child.length) {
    window.openTab = tabId;
  }
}

/**
 *  Returns the DOM representation of the given Widget Tree structure
 *  that is to be rendered by ReactDOM.
 *
 *  @param {Window} object WidgetTree to render
 *  @param {Function} callback a callback function for when the WidgetTree
 *           is modified.
 *
 *  @return {React.Component} DOM representation of the WidgetTree
 */
export function renderWindows(root, callback, currNode = root) {
  if (currNode !== null && currNode instanceof Window) {
    if(currNode.hasLeafChildren) {
      return (
        <div className="widget-window">
          {currNode.child.map((c, index) => {
            return (
              <div className="widget-tab" onClick={() => {
                setTab(root, currNode.WIN_ID, index);
                callback(root);
              }}>
                {c.type}
              </div>
            );
          })}
          {generateComponent(currNode.child[currNode.openTab])}
        </div>
      );
    } else {
      return (
        <div className="window-wrapper">
          {currNode.child.map((c) => {
            return renderWindows(root, callback, c);
          })}
        </div>
      )
    }
  }
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
  return WIDGET_DICT[component.type];
}

export default {
  Window,
  Leaf,
  add,
  remove,
  get,
  setTab,
  renderWindows,
  generateComponent,
};
