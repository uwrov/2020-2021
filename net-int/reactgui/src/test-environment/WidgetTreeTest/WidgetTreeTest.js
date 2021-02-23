let WINDOW_COUNT = 0;

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
