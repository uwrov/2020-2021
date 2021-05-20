import React from "react";

import { WIDGET_DICT } from "./WidgetLib.js";

import "./widgets.css";

let WINDOW_COUNT = [0, 0];

export class WidgetTree {
  constructor(updateCallback) {
    this.updateState = updateCallback;
    this.root = null;
  }
  // object: Object to be added to. Must be a window.
  // node: Node to be added. Must be a window or a leaf
  // If adding a window to a parent window, adds the window to the parent window (Case 2). If the
  // parent window had leaves, adds a new window between the parent window and the leaves (Case 1)
  // If adding a leaf to a window, adds the leaf to the window if it has other leafs (Case 3)
  // Otherwise, finds the leftmost window with leaves and adds the leaf to that window (Case 4)
  add(node, windowId, object = this.root, isTest = 0) {
    if (object == null) {
      object = new Window(isTest);
      if (node instanceof Leaf) object.hasLeafChildren = true;
    }
    if (object instanceof Window) {
      if (node instanceof Window) {
        // Case 1
        if (object.hasLeafChildren) {
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
      } else if (node instanceof Leaf) {
        // Case 3
        if (object.hasLeafChildren) {
          object.child.push(node);
        } else {
          // Case 4
          if (object.child.length > 0) {
            object.child[0] = add(object.child[0], node);
          } else {
            object.hasLeafChildren = true;
            object.child.push(node);
          }
        }
      } else {
        throw new Error("The second param must be of type Window or Leaf");
      }
    } else {
      throw new Error("The first param must be of type Window");
    }
    return object;
  }

  get(windowId, componentId, object = this.root) {
    if (object instanceof Window) {
      if (!object.hasLeafChildren) {
        if (componentId !== -1) {
          for (let i = 0; i < object.child.length; i++) {
            let obj = get(windowId, componentId, object.child[i]);
            if (obj !== null) return obj;
          }
        } else if (object.WIN_ID === windowId) return object;
        return null;
      } else {
        if (object.WIN_ID === windowId && object.child.length > componentId) {
          if (componentId < 0) return object;
          else return object.child[componentId];
        }
        return null;
      }
    }
    return null;
  }
}

export class Window {
  constructor(isTest = 0) {
    this.WIN_ID = WINDOW_COUNT[isTest]++;
    this.hasLeafChildren = false;
    this.child = [];
    this.openTab = 0;
    this.height = 100;
    this.width = 100;
    this.style = {
      width: "100px",
      height: "100px",
    };
    this.drag = false;
  }

  updateStyle() {
    this.style = {
      width: this.width + "px",
      height: this.height + "px",
    };
  }
}

export class Leaf {
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

// Object must be a Window
// TODO comment all cases.
export function remove(object, windowID, componentID, isRoot = true) {
  if (object instanceof Window) {
    if (object.hasLeafChildren) {
      if (object.WIN_ID === windowID) {
        object.child.splice(componentID, 1);
      }
      if (object.child.length === 0 && !isRoot) {
        return null;
      }
    } else {
      //Call remove on all this window's children windows
      for (let i = 0; i < object.child.length; i++) {
        let removed = remove(object.child[i], windowID, componentID, false);
        // Delete a child if it becomes null
        if (removed === null) {
          object.child.splice(0, 1);
        } else {
          object.child[i] = removed;
        }
        // Remove the intermediary window if a window has only 1 window child
        if (object.child.length === 1) {
          return object.child[0];
        }
      }
    }
  } else {
    throw "object must be a Window";
  }
  return object;
}

export function setTab(object, windowId, tabId) {
  let window = get(object, windowId, -1);
  if (window !== null && tabId < window.child.length) {
    window.openTab = tabId;
  }
}

/**
 * Resizing Widget Tree
 *
 *
 *
 */
let oldWidth, oldHeight;
export function averageSize(object, width, height, stackWindows = false) {
  if (oldWidth === undefined && oldHeight === undefined) {
    oldWidth = width;
    oldHeight = height;
  }
  object.width = width;
  object.height = height;
  object.updateStyle();
  if (!object.hasLeafChildren && object.child.length > 0) {
    let num = object.child.length;
    let newW = width;
    let newH = height;
    if (stackWindows) newH = height / num;
    else newW = width / num;
    object.child.forEach((object) => {
      averageSize(object, newW, newH, !stackWindows);
    });
  }
}

export function updateSizes(
  object,
  offset,
  updateWidths = true,
  avgOffsetLayer = false
) {
  if (updateWidths) {
    object.width += offset;
  } else {
    object.height += offset;
  }
  object.updateStyle();
  if (!object.hasLeafChildren && object.child.length > 0) {
    object.child.forEach((child) => {
      updateSizes(
        child,
        avgOffsetLayer ? child / child.child.length : offset,
        updateWidths,
        !avgOffsetLayer
      );
    });
  }
}

export function createDragSection(
  root,
  callback,
  currNode,
  isSideBySide,
  adjNode
) {
  if (adjNode !== null) {
    return (
      <div
        className={isSideBySide ? "drag-section-right" : "drag-section-bottom"}
        onMouseDown={(event) => {
          onMouseDownResize(event, currNode, adjNode, isSideBySide);
        }}
      ></div>
    );
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
export function renderWindows(
  root,
  callback,
  currNode = root,
  isSideBySide = false,
  adjNode = null
) {
  if (currNode !== null && currNode instanceof Window) {
    if (currNode.hasLeafChildren) {
      return generateWidgetWindow(
        root,
        callback,
        currNode,
        isSideBySide,
        adjNode
      );
    } else {
      return generateWidgetWrapper(
        root,
        callback,
        currNode,
        isSideBySide,
        adjNode
      );
    }
  }
}

function generateWidgetWindow(
  root,
  callback,
  currNode = root,
  isSideBySide = false,
  adjNode = null
) {
  return (
    <div
      className="widget-window"
      style={currNode.style}
      onMouseMove={(event) => {
        onMouseMove(event, callback, root);
      }}
      onMouseUp={(event) => {
        onMouseUp(add(new Window(), currNode), callback, root);
      }}
    >
      {createDragSection(root, callback, currNode, isSideBySide, adjNode)}
      {generateAllTabs(currNode, root, callback)}
      <div className="widget-content">
        {generateComponent(currNode.child[currNode.openTab])}
      </div>
    </div>
  );
}

function generateWidgetWrapper(
  root,
  callback,
  currNode = root,
  isSideBySide = false,
  adjNode = null
) {
  return (
    <div className="window-wrapper" style={currNode.style}>
      {currNode.child.map((curNode, index, arr) => {
        if (index !== arr.length - 1) {
          // console.log("paired widgets",arr[index+1], curNode, !isSideBySide);
          return renderWindows(
            root,
            callback,
            curNode,
            !isSideBySide,
            arr[index + 1]
          );
        } else {
          // console.log("path2",arr[index], curNode);
          return renderWindows(root, callback, curNode, !isSideBySide);
        }
      })}
    </div>
  );
}

function generateAllTabs(currWindow, root, callback) {
  return (
    <div
      className="tab-section"
      onMouseUp={(event) => {
        onMouseUp(currWindow, callback, root);
      }}
    >
      <div className="grouped-tabs">
        {currWindow.child.map((c, index) => {
          if (currWindow.openTab == index) {
            return generateFocusedTab(c, currWindow, index, root, callback);
          } else {
            return generateUnfocusedTab(c, currWindow, index, root, callback);
          }
        })}
      </div>
    </div>
  );
}

function generateUnfocusedTab(currTab, currWindow, tabIndex, root, callback) {
  return generateSingleTab(
    "widget-tab",
    currTab,
    currWindow,
    tabIndex,
    root,
    callback
  );
}

function generateFocusedTab(currTab, currWindow, tabIndex, root, callback) {
  return generateSingleTab(
    "widget-tab widget-tab-focused",
    currTab,
    currWindow,
    tabIndex,
    root,
    callback
  );
}

function generateSingleTab(
  className,
  currTab,
  currWindow,
  tabIndex,
  root,
  callback
) {
  return (
    <div
      className={className}
      onClick={() => {
        setTab(root, currWindow.WIN_ID, tabIndex);
        callback(root);
      }}
      onMouseDown={(event) => {
        onMouseDownRelocate(event, currWindow.WIN_ID, tabIndex, currTab);
      }}
    >
      <a>{currTab.type}</a>
      <span
        className="tab-exit-button"
        onClick={() => {
          let newRoot = remove(root, currWindow.WIN_ID, tabIndex);
          callback(newRoot);
          console.log("Removing: " + currWindow.WIN_ID + ", " + tabIndex);
          console.log(newRoot);
        }}
      >
        &times;
      </span>
    </div>
  );
}

export function handleResize(root, callback) {
  let widthChange = window.innerWidth - oldWidth;
  let heightChange = window.innerHeight - 80 - oldHeight;
  oldWidth = window.innerWidth;
  oldHeight = window.innerHeight - 80;
  resizeWidthHeight(root, widthChange, heightChange);
  callback(root);
}

function resizeWidthHeight(
  object,
  widthChange,
  heightChange,
  stackWindows = true
) {
  object.width += widthChange;
  object.height += heightChange;
  object.updateStyle();
  if (!object.hasLeafChildren && object.child.length > 0) {
    object.child.forEach((child) => {
      resizeWidthHeight(
        child,
        stackWindows ? widthChange / object.child.length : widthChange,
        stackWindows ? heightChange : heightChange / object.child.length,
        !stackWindows
      );
    });
  }
}
/**
 *
 *                    RESIZE & TAB RELOCATION HANDLERS
 *
 */
let dragWindow = null;
let adjWindow = null;
let updatesWidth = null;
let selectedWindowID = null;
let selectedTab = null;
let selectedObject = null;
let removed = true;

function onMouseDownResize(event, curNode, adjNode, isSideBySide) {
  dragWindow = curNode;
  adjWindow = adjNode;
  updatesWidth = isSideBySide;
}

function onMouseDownRelocate(event, curWindowID, tabIdx, currTab) {
  selectedWindowID = curWindowID;
  selectedTab = tabIdx;
  selectedObject = currTab;
  removed = false;
}

function onMouseMove(event, callback, root) {
  if (dragWindow !== null) {
    updateSizes(
      dragWindow,
      updatesWidth ? event.movementX : event.movementY,
      updatesWidth
    );
    updateSizes(
      adjWindow,
      updatesWidth ? -event.movementX : -event.movementY,
      updatesWidth
    );
    callback(root); //rerendering widgets
  } else if (!removed) {
    console.log("moved", removed);
    let newRoot = remove(root, selectedWindowID, selectedTab);
    callback(newRoot);
    removed = true;
  }
}

function onMouseUp(toAdd, callback, root) {
  if (dragWindow !== null) {
    dragWindow = null;
    adjWindow = null;
  } else if (selectedWindowID !== null) {
    if (removed) {
      add(toAdd, selectedObject);
    }
    removed = true;
    callback(root);
    selectedWindowID = null;
    selectedTab = null;
    selectedObject = null;
  }
}

function onMouseUpRelocate(toAdd) {
  if (selectedWindowID !== null) {
    selectedWindowID = null;
    selectedTab = null;
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
  if (component !== null && component !== undefined) {
    return WIDGET_DICT(component);
  }
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
  averageSize,
  handleResize,
};
