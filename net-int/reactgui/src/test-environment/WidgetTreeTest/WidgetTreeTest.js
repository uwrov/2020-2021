import { Window, Leaf, add, remove, get, setTab } from "../../datastructs/WidgetTree.js";

let WINDOW_COUNT = 0;

export function testAdd() {
   let root = new Window();
   let testRoot = new Window(1);

   // Tests adding a leaf to the root
   let windowLeaf = new Window();
   windowLeaf.hasLeafChildren = true;
   windowLeaf.push(new Leaf());
   root.child.push(windowLeaf);
   add(testRoot, new Leaf(),1);
   Assert(root, testRoot);

   // Tests adding a new window with a leaf to a root
   let windowLeaf2 = new Window();
   windowLeaf.hasLeafChildren = true;
   windowLeaf.push(new Leaf());
   root.child.push(windowLeaf2);
   let testWindowLeaf2 = new Window();
   windowLeaf.hasLeafChildren = true;
   windowLeaf.push(new Leaf());
   add(testRoot, testWindowLeaf2,1);
   Assert(root, testRoot);

   //
}

export function testRemove() {

}

export function testGet() {

}

export function testSetTab() {

}

export function testEquals() {
   let root = new Window();
   let testRoot = new Window(1);

   Assert(root, testRoot);

   root.child.push(new Window());
   testRoot.child.push(new Window(1));

   Assert(root, testRoot);
}


export function Assert(tree1, tree2) {
   if(treeEquals(tree1, tree2)) {
      console.log("Pass: Tree1 === Tree2");
   } else {
      console.log("Fail: Tree1 !== Tree2");
   }
}

export function treeEquals(tree1, tree2) {
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

export default { testAdd, testRemove, testSetTab, testEquals}
