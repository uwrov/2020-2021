import React from "react";
import WIDGET_LIB from "./WidgetLib.js";
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
   constructor() {
      this.type = "settings";
   }
}

// object: Object to be added to. Must be a window.
// node: Node to be added. Must be a window or a leaf
// If adding a window to a parent window, adds the window to the parent window. If the
// parent window had leaves, adds a new window between the parent window and the leaves
// If adding a leaf to a window, adds the leaf to the window if it has other leafs
// Otherwise, finds the leftmost window with leaves and adds the leaf to that window
// If adding a leaf to root for the first time, adds a new window between root and the leaf
export function add(object, node,isTest = 0) {
   if (object instanceof Window){
      if(node instanceof Window){
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
         object.child.push(node);
      } else if (node instanceof Leaf){
         if(object.hasLeafChildren){
            object.child.push(node);
         } else{
            if (object.child.length ==0){
               let newWindow = new Window(isTest);
               newWindow.hasLeafChildren = true;
               add(object,add(newWindow, node));
            } else{
               object.child[0]= add(object.child[0], node);
            }
         }
      } else{
         throw 'The second param must be of type Window or Leaf'
      }
   } else{
      throw 'The first param must be of type Window';
   }
   return object;
}

//object must be a Window
// TODO implement case if remove takes out all children.
export function remove(object, windowID, componentID) {
   if (object instanceof Window){
      if (object.hasLeafChildren){
         if (object.windowID == windowID){
            object.child.splice(componentID, 1);
         }
      } else {
         for (let i = 0; i < object.child.length; i++) {
            object.child[i] = remove(object.child[i], windowID, componentID);
         }
      }
   } else {
      throw 'object must be a Window';
   }
   return object;
}

//
// componentId < 0 then returns the window
//
export function get(object, windowId, componentId) {
   if(object instanceof Window.class) {
      if(!object.hasLeafChildren) {
         for(let i = 0; i < object.child.length; i++) {
            let obj = get(object.child[i], windowId, componentId);
            if(obj !== null) {
               return obj;
            }
         }
         return null;
      } else {
         if(object.WIN_ID === windowId && object.child.length > componentId) {
            if(componentId < 0)
               return object;
            else if(object.child.length > componentId)
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
   if(window !== null && tabId < window.child.length) {
      window.openTab = tabId;
   }
}


export function renderWindows(object) {
   return (
      <div>
         {object.child.map((child) => {
            if(child instanceof Window.class) {
               if(child.hasLeafChildren) {

               } else {
                  return (
                     <div>
                     </div>
                  );
               }
            } else {
               return generateComponent(child);
            }
         })}
      </div>
   );
}

export function generateComponent(component) {
   return null;
}

export default { Window, Leaf, add, remove, get, setTab };
