let WINDOW_COUNT = 0;

class Window {
   constructor() {
      this.WIN_ID = WINDOW_COUNT++;
      this.hasLeafChildren = false;
      this.child = [];
      this.openTab = 0;
      this.style = {};
   }
}

class Leaf {
   constructor() {
      this.type = "settings";
   }
}


function add(object, node) {

}

function remove(object, node) {

}

function get(object, windowId, componentId) {
   if(object instanceof Window.class) {
      if(!object.hasLeafChildren) {
         for(let i = 0; i < object.child.length; i++) {
            let obj = get(object.child[i], windowId, componentId);
            if(obj instanceof Leaf.class) {
               return obj;
            } else if (obj instanceof Number) {
               windowId = windowId - obj;
            } else {
               throw Error;
            }
         }
         return null;
      } else {
         if(object.child.length < windowId) {
            return object.child.length - windowId;
         } else {
            return object.child[windowId];
         }
      }
   } else {
      return null;
   }
}

function setTab(object, windowId, tabId) {
   if(object instanceof Window.class && windowId >= 0) {
      if(!object.hasLeafChildren) {
         let winCount = 0;
         for(let i = 0; i < object.child.length; i++) {
            let c = object.child[i];
            let subCount = setTab(c, windowId - winCount, tabId);
            winCount += subCount;
         }
         return winCount;
      } else {
         if(windowId == 0) {
            object.openTab = tabId;
         }
         return 1;
      }
   }
   return 0;
}

/**
function renderWindows(object) {
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
*/
function testAdd() {


}

function testRemove() {

}

function testGet() {

}

function testSetTab() {

}

function testEquals() {
   let root = new Window();
   let root2 = new Window();

   Assert(root, root2);

   root.child.push(new Window());
   root2.child.push(new Window());

   Assert(root, root2);
}


function Assert(tree1, tree2) {
   if(treeEquals(tree1, tree2)) {
      console.log("Pass: Tree1 === Tree2");
   } else {
      console.log("Fail: Tree1 !== Tree2");
   }
}

function treeEquals(tree1, tree2) {
   if(tree1.WIN_ID === tree2.WIN_ID &&
                        tree1.hasLeafChildren === tree2.hasLeafChildren &&
                        tree1.openTab === tree2.openTab &&
                        tree1.child.length === tree2.child.length) {
      for(let i = 0; i < tree1.child.length; i++) {
         if(!treeEquals(tree1.child[i], tree2.child[i])) {
            return false;
         }
      }
      return true;
   }
   return false;
}

testEquals();
