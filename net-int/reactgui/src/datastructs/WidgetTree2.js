import React from "react";

import { WIDGET_DICT } from "./WidgetLib.js";

import "./widgets.css";

let WINDOW_COUNT = [0, 0];

export class WidgetTree {
  constructor(updateCallback) {
    this.updateState = updateCallback;
    this.root = null;
  }

  add(node, windowId, isTest = 0) {
    if(this.root.WIN_ID != windowId)
      this.root = this.addStep(node, windowId, this.root, isTest);
    else
      this.root = addNode(node, this.root, isTest);
    this.update()
  }

  remove(windowId, componentId) {
    this.root = removeNode(windowId, componentId);
    this.update();
  }

  setTab(windowId, tabId) {
    let window = get(windowId, -1);
    if (window !== null && tabId < window.child.length) {
      window.openTab = tabId;
    }
    this.update();
  }

  get(windowId, componentId, object = this.root) {
    if (object instanceof Window) {
      if (!object.hasLeafChildren) {
        if (componentId !== -1) {
          for (let i = 0; i < object.child.length; i++) {
            let obj = this.get(windowId, componentId, object.child[i]);
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

  update() {
    this.updateState(this.root);
  }

  createDragSection(currNode, isSideBySide, adjNode) {
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

  renderWindows(currNode = this.root, isSideBySide = false, adjNode = null) {
    if (currNode !== null && currNode instanceof Window) {
      if (currNode.hasLeafChildren) {
        return this.generateWidgetWindow(
          currNode,
          isSideBySide,
          adjNode
        );
      } else {
        return this.generateWidgetWrapper(
          currNode,
          isSideBySide,
          adjNode
        );
      }
    }
  }

  generateWidgetWindow(currNode = this.root, isSideBySide = false, adjNode = null) {
    return (
      <div
        className="widget-window"
        style={currNode.style}
        onMouseMove={(event) => {
          onMouseMove(event, this.updateState, this.root);
        }}
        onMouseUp={(event) => {
          onMouseUp(add(new Window(), currNode), this.updateState, this.root);
        }}
      >
        {this.createDragSection(currNode, isSideBySide, adjNode)}
        {this.generateAllTabs(currNode)}
        <div className="widget-content">
          {generateComponent(currNode.child[currNode.openTab])}
        </div>
      </div>
    );
  }

  generateWidgetWrapper(currNode = root, isSideBySide = false, adjNode = null) {
    return (
      <div className="window-wrapper" style={currNode.style}>
        {currNode.child.map((curNode, index, arr) => {
          if (index !== arr.length - 1) {
            // console.log("paired widgets",arr[index+1], curNode, !isSideBySide);
            return this.renderWindows(
              curNode,
              !isSideBySide,
              arr[index + 1]
            );
          } else {
            // console.log("path2",arr[index], curNode);
            return this.renderWindows(curNode, !isSideBySide);
          }
        })}
      </div>
    );
  }

  generateAllTabs(currWindow, root, callback) {
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
              return this.generateFocusedTab(c, currWindow, index, root, callback);
            } else {
              return this.generateUnfocusedTab(c, currWindow, index, root, callback);
            }
          })}
        </div>
      </div>
    );
  }

  generateUnfocusedTab(currTab, currWindow, tabIndex) {
    return generateSingleTab(
      "widget-tab",
      currTab,
      currWindow,
      tabIndex
    );
  }

  generateFocusedTab(currTab, currWindow, tabIndex) {
    return generateSingleTab(
      "widget-tab widget-tab-focused",
      currTab,
      currWindow,
      tabIndex
    );
  }

  generateSingleTab(className, currTab, currWindow, tabIndex) {
    return (
      <div
        className={className}
        onClick={() => {
          this.setTab(currWindow.WIN_ID, tabIndex);
          this.update();
        }}
        onMouseDown={(event) => {
          onMouseDownRelocate(event, currWindow.WIN_ID, tabIndex, currTab);
        }}
      >
        <a>{currTab.type}</a>
        <span
          className="tab-exit-button"
          onClick={() => {
            this.root = this.remove(currWindow.WIN_ID, tabIndex);
            this.update();
            console.log("Removing: " + currWindow.WIN_ID + ", " + tabIndex);
          }}
        >
          &times;
        </span>
      </div>
    );
  }

  handleResize() {
    let widthChange = window.innerWidth - oldWidth;
    let heightChange = window.innerHeight - 80 - oldHeight;
    oldWidth = window.innerWidth;
    oldHeight = window.innerHeight - 80;
    resizeWidthHeight(this.root, widthChange, heightChange);
    this.update();
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




/**
*
*             WIDGETTREE NODE MANIPULATION UTILITY
*
*/
function addStep(node, windowId, currNode, isTest = 0) {
  for(let i=0; i < currNode.child.length; i++) {
    let temp = currNode.child[i];
    if(temp.WIN_ID == windowId) {
      currNode.child[i] = addNode(node, temp, isTest);
    } else {
      currNode.child[i] = addStep(node, temp, isTest);
    }
  }
  return currNode;
}

// object: Object to be added to. Must be a window.
// node: Node to be added. Must be a window or a leaf
// If adding a window to a parent window, adds the window to the parent window (Case 2). If the
// parent window had leaves, adds a new window between the parent window and the leaves (Case 1)
// If adding a leaf to a window, adds the leaf to the window if it has other leafs (Case 3)
// Otherwise, finds the leftmost window with leaves and adds the leaf to that window (Case 4)
function addNode(node, object = this.root, isTest = 0) {
  if (object == null) {
    object = new Window(isTest);
    if (node instanceof Leaf) object.hasLeafChildren = true;
  }
  if (object instanceof Window) {
    if (node instanceof Window) {
      // Case 1
      if (object.hasLeafChildren) {
        let newWindow = new Window(isTest);
        newWindow.child.push(object);
        object = newWindow;
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
          object.child[0] = this.addNode(node, object.child[0]);
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

// Object must be a Window
// If windowId points to window that has no leafchildren then will delete the
// entire window regardless of the componentId
function removeNode(windowId, componentId) {
  if (object instanceof Window) {
    if (object.WIN_ID === windowId) {
      if(object.hasLeafChildren && componentId >= 0)
        object.child.splice(componentID, 1);
      else
        return null;
    } else if(!object.hasLeafChildren) {
      //Call remove on all this window's children windows
      for (let i = 0; i < object.child.length; i++) {
        let removed = remove(windowID, componentID, object.child[i]);
        // Delete a child if it becomes null
        if (removed === null) {
          object.child.splice(i, 1);
          i--;
        } else {
          object.child[i] = removed;
        }
      }
      // Remove the intermediary window if a window has only 1 window child
      if (object.child.length === 1) {
        return object.child[0];
      } else if(object.child.length === 0) {
        return null;
      }
    }
  } else {
    throw "object must be a Window";
  }
  return object;
}



/**
 *
 *                      WINDOW RESIZE UTILITY
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
  WidgetTree,
  Window,
  Leaf,
  generateComponent,
  averageSize,
  updateSizes
};
