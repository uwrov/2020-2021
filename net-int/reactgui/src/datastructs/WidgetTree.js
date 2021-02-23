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

export function add(object, node) {

}

export function remove(object, node) {

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
