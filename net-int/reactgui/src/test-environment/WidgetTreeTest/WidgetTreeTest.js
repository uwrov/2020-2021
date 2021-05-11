import {
  Window,
  Leaf,
  add,
  remove,
  get,
  setTab,
} from "../../datastructs/WidgetTree.js";

let root = new Window();
root.hasLeafChildren = true;
let testRoot = new Window(1);
testRoot.hasLeafChildren = true;

function testAdd() {
   // Tests adding a leaf to the root
   root.child.push(new Leaf());
   add(testRoot, new Leaf(),1);
   assert(root, testRoot);

   // Tests adding a new window with a leaf to a root
   let windowLeaf0 = new Window();
   windowLeaf0.hasLeafChildren = true;
   windowLeaf0.child.push(new Leaf());
   root.child.push(windowLeaf0);
   let windowLeaf1 = new Window();
   windowLeaf1.hasLeafChildren = true;
   windowLeaf1.child.push(root.child[0]);
   root.child[0] = windowLeaf1;
   root.hasLeafChildren = false;
   let testWindowLeaf0 = new Window(1);
   testWindowLeaf0.hasLeafChildren = true;
   testWindowLeaf0.child.push(new Leaf());
   add(testRoot, testWindowLeaf0,1);
   assert(root, testRoot);

   //Tests adding a new window with a leaf to a window with leaf children
   let rootChildWindow =root.child[0]
   let windowLeaf2 = new Window();
   windowLeaf2.hasLeafChildren = true;
   windowLeaf2.child.push(new Leaf());
   let windowLeaf3 = new Window();
   windowLeaf3.hasLeafChildren = true;
   windowLeaf3.child.push(rootChildWindow.child[0]);
   rootChildWindow.child[0] = windowLeaf3;
   rootChildWindow.child.push(windowLeaf2);
   rootChildWindow.hasLeafChildren = false;

   let testWindowLeaf2 = new Window(1);
   testWindowLeaf2.hasLeafChildren = true;
   testWindowLeaf2.child.push(new Leaf());
   add(testRoot.child[0], testWindowLeaf2,1);
   assert(root, testRoot);

   //Tests that adding a leaf to root falls to the first window (in-order)
   // that has leaf children
   root.child[0].child[0].child.push(new Leaf());
   add(testRoot,new Leaf(), 1);
   assert(root,testRoot);
}

function testRemove() {
   //root.remove(root,4,0);
}

function testGet() {

}

function testSetTab() {

}

function testEquals() {
   let root = new Window();
   let testRoot = new Window(1);

   assert(root, testRoot);

   root.child.push(new Window());
   testRoot.child.push(new Window(1));
  
   assert(root, testRoot);
}


function assert(tree1, tree2) {
   if(treeEquals(tree1, tree2)) {
      console.log("Pass: Tree1 === Tree2");
   } else {
      console.log("Fail: Tree1 !== Tree2");
      throw new Error("trees did not equal!")
   }
}

function treeEquals(tree1, tree2) {
   if (tree1 instanceof Window && tree2 instanceof Window) {
      if (tree1.WIN_ID === tree2.WIN_ID &&
          tree1.hasLeafChildren === tree2.hasLeafChildren &&
          tree1.openTab === tree2.openTab &&
          tree1.child.length === tree2.child.length) {
         for (let i = 0; i < tree1.child.length; i++) {
            if (!treeEquals(tree1.child[i], tree2.child[i])) {
               return false;
            }
         }
         return true;
      }
      return false;
   } else if(tree1 instanceof Leaf && tree2 instanceof Leaf){
      if (tree1.type === tree2.type){
         return true;
      }
      return false;
   } else {
      return false;
   }
}

export function runAllTests(){
   testAdd();
   testRemove();
   testGet();
   testSetTab();
   testEquals();
}

export default {runAllTests}