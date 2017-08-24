//--------------------------------------------------------------------------------------------------
/**
    Qu3e Physics Engine v1.01 - Unofficial C# Version with modifications

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using static Qu3e.Settings;

namespace Qu3e
{
    public class DynamicAABBTree
    {
        public DynamicAABBTree()
        {
            Root = Node.Null;

            Capacity = 1024;
            Count = 0;
            Nodes = new Node[Capacity];

            AddToFreeList(0);
        }

        // Provide tight-AABB
        public int Insert(AABB aabb, object userData)
        {
            int id = AllocateNode();

            // Fatten AABB and set height/userdata
            Nodes[id].aabb = aabb;
            FattenAABB(ref Nodes[id].aabb);
            Nodes[id].userData = userData;
            Nodes[id].height = 0;

            InsertLeaf(id);

            return id;
        }
        public void Remove(int id)
        {
            Assert(id >= 0 && id < Capacity);
            Assert(Nodes[id].IsLeaf());

            RemoveLeaf(id);
            DeallocateNode(id);
        }
        public bool Update(int id, AABB aabb)
        {
            Assert(id >= 0 && id < Capacity);
            Assert(Nodes[id].IsLeaf());

            if (Nodes[id].aabb.Contains(aabb))
                return false;

            RemoveLeaf(id);

            Nodes[id].aabb = aabb;
            FattenAABB(ref Nodes[id].aabb);

            InsertLeaf(id);

            return true;

        }

        public object GetUserData(int id)
        {
            Assert(id >= 0 && id < Capacity);

            return Nodes[id].userData;
        }
        public AABB GetFatAABB(int id)
        {
            Assert(id >= 0 && id < Capacity);

            return Nodes[id].aabb;
        }
        public void Render(Render render)
        {
            if (Root != Node.Null)
            {
                render.SetPenColor(0.5f, 0.5f, 1.0f);
                RenderNode(render, Root);
            }
        }

        int[] c_stack = new int[1024];

        public void Query(ITreeCallback cb, AABB aabb)
        {
            int sp = 1;

            c_stack[0] = Root;

            while (sp > 0)
            {
                int id = c_stack[--sp];

                Node n = Nodes[id];
                if (AABB.AABBtoAABB(aabb, n.aabb))
                {
                    if (n.IsLeaf())
                    {
                        if (!cb.TreeCallback(id))
                            return;
                    }
                    else
                    {
                        c_stack[sp++] = n.left;
                        c_stack[sp++] = n.right;
                    }
                }
            }
        }

        public void Query(ITreeCallback cb, RaycastData rayCast)
        {
            double k_epsilon = 1e-6;
            int k_stackCapacity = 256;
            int[] stack = new int[k_stackCapacity];
            int sp = 1;

            stack[0] = Root;

            Vec3 p0 = rayCast.start;
            Vec3 p1 = p0 + rayCast.dir * rayCast.t;

            while (sp > 0)
            {
                // k_stackCapacity too small
                Assert(sp < k_stackCapacity);

                int id = stack[--sp];

                if (id == Node.Null)
                    continue;

                Node n = Nodes[id];

                Vec3 e = n.aabb.max - n.aabb.min;
                Vec3 d = p1 - p0;
                Vec3 m = p0 + p1 - n.aabb.min - n.aabb.max;

                double adx = Math.Abs(d.x);

                if (Math.Abs(m.x) > e.x + adx)
                    continue;

                double ady = Math.Abs(d.y);

                if (Math.Abs(m.y) > e.y + ady)
                    continue;

                double adz = Math.Abs(d.z);

                if (Math.Abs(m.z) > e.z + adz)
                    continue;

                adx += k_epsilon;
                ady += k_epsilon;
                adz += k_epsilon;

                if (Math.Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady)
                    continue;

                if (Math.Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx)
                    continue;

                if (Math.Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx)
                    continue;

                if (n.IsLeaf())
                {
                    if (!cb.TreeCallback(id))
                        return;
                }

                else
                {
                    stack[sp++] = n.left;
                    stack[sp++] = n.right;
                }
            }

        }

        // For testing
        public void Validate()
        {
            // Verify free list
            int freeNodes = 0;
            int index = FreeList;

            while (index != Node.Null)
            {
                Assert(index >= 0 && index < Capacity);
                index = Nodes[index].next;
                ++freeNodes;
            }

            Assert(Count + freeNodes == Capacity);

            // Validate tree structure
            if (Root != Node.Null)
            {
                Assert(Nodes[Root].parent == Node.Null);

#if _DEBUG
                ValidateStructure(Root);
#endif
            }
        }

        public class Node
        {
            public bool IsLeaf()
            {
                // The right leaf does not use the same memory as the userdata,
                // and will always be Null (no children)
                return right == Null;
            }

            // Fat AABB for leafs, bounding AABB for branches
            public AABB aabb;

            public int parent;
            public int next; // free list

            // Child indices

            public int left;
            public int right;

            // Since only leaf nodes hold userdata, we can use the
            // same memory used for left/right indices to store
            // the userdata void pointer
            public object userData;

            // leaf = 0, free nodes = -1
            public int height;

            static public int Null = -1;
        }

        int AllocateNode()
        {
            if (FreeList == Node.Null)
            {
                Capacity *= 2;
                Node[] newNodes = new Node[Capacity];
                Array.Copy(Nodes, newNodes, Count);
                Nodes = newNodes;

                AddToFreeList(Count);
            }

            int freeNode = FreeList;
            FreeList = Nodes[FreeList].next;
            Nodes[freeNode].height = 0;
            Nodes[freeNode].left = Node.Null;
            Nodes[freeNode].right = Node.Null;
            Nodes[freeNode].parent = Node.Null;
            Nodes[freeNode].userData = null;
            ++Count;
            return freeNode;

        }

        void DeallocateNode(int index)
        {
            Assert(index >= 0 && index < Capacity);

            Nodes[index].next = FreeList;
            Nodes[index].height = Node.Null;
            FreeList = index;

            --Count;
        }

        int Balance(int iA)
        {
            Node A = Nodes[iA];

            if (A.IsLeaf() || A.height == 1)
                return iA;

            /*      A
                  /   \
                 B     C
                / \   / \
               D   E F   G
            */

            int iB = A.left;
            int iC = A.right;
            Node B = Nodes[iB];
            Node C = Nodes[iC];

            int balance = C.height - B.height;

            // C is higher, promote C
            if (balance > 1)
            {
                int iF = C.left;
                int iG = C.right;
                Node F = Nodes[iF];
                Node G = Nodes[iG];

                // grandParent point to C
                if (A.parent != Node.Null)
                {
                    if (Nodes[A.parent].left == iA)
                        Nodes[A.parent].left = iC;
                    else
                        Nodes[A.parent].right = iC;
                }
                else
                    Root = iC;

                // Swap A and C
                C.left = iA;
                C.parent = A.parent;
                A.parent = iC;

                // Finish rotation
                if (F.height > G.height)
                {
                    C.right = iF;
                    A.right = iG;
                    G.parent = iA;
                    A.aabb = AABB.Combine(B.aabb, G.aabb);
                    C.aabb = AABB.Combine(A.aabb, F.aabb);

                    A.height = 1 + Math.Max(B.height, G.height);
                    C.height = 1 + Math.Max(A.height, F.height);
                }

                else
                {
                    C.right = iG;
                    A.right = iF;
                    F.parent = iA;
                    A.aabb = AABB.Combine(B.aabb, F.aabb);
                    C.aabb = AABB.Combine(A.aabb, G.aabb);

                    A.height = 1 + Math.Max(B.height, F.height);
                    C.height = 1 + Math.Max(A.height, G.height);
                }

                return iC;
            }

            // B is higher, promote B
            else if (balance < -1)
            {
                int iD = B.left;
                int iE = B.right;
                Node D = Nodes[iD];
                Node E = Nodes[iE];

                // grandParent point to B
                if (A.parent != Node.Null)
                {
                    if (Nodes[A.parent].left == iA)


                        Nodes[A.parent].left = iB;
                    else
                        Nodes[A.parent].right = iB;
                }

                else
                    Root = iB;

                // Swap A and B
                B.right = iA;
                B.parent = A.parent;
                A.parent = iB;

                // Finish rotation
                if (D.height > E.height)
                {
                    B.left = iD;
                    A.left = iE;
                    E.parent = iA;
                    A.aabb = AABB.Combine(C.aabb, E.aabb);
                    B.aabb = AABB.Combine(A.aabb, D.aabb);

                    A.height = 1 + Math.Max(C.height, E.height);
                    B.height = 1 + Math.Max(A.height, D.height);
                }

                else
                {
                    B.left = iE;
                    A.left = iD;
                    D.parent = iA;
                    A.aabb = AABB.Combine(C.aabb, D.aabb);
                    B.aabb = AABB.Combine(A.aabb, E.aabb);

                    A.height = 1 + Math.Max(C.height, D.height);
                    B.height = 1 + Math.Max(A.height, E.height);
                }

                return iB;
            }

            return iA;

        }

        void InsertLeaf(int id)
        {
            if (Root == Node.Null)
            {
                Root = id;
                Nodes[Root].parent = Node.Null;
                return;
            }

            // Search for sibling
            int searchIndex = Root;
            AABB leafAABB = Nodes[id].aabb;
            while (!Nodes[searchIndex].IsLeaf())
            {
                // Cost for insertion at index (branch node), involves creation
                // of new branch to contain this index and the new leaf
                AABB combined = AABB.Combine(leafAABB, Nodes[searchIndex].aabb);
                double combinedArea = combined.SurfaceArea();
                double branchCost = 2 * combinedArea;

                // Inherited cost (surface area growth from heirarchy update after descent)
                double inheritedCost = 2 * (combinedArea - Nodes[searchIndex].aabb.SurfaceArea());

                int left = Nodes[searchIndex].left;
                int right = Nodes[searchIndex].right;

                // Calculate costs for left/right descents. If traversal is to a leaf,
                // then the cost of the combind AABB represents a new branch node. Otherwise
                // the cost is only the inflation of the pre-existing branch.
                double leftDescentCost;
                if (Nodes[left].IsLeaf())
                    leftDescentCost = AABB.Combine(leafAABB, Nodes[left].aabb).SurfaceArea() + inheritedCost;
                else
                {
                    double inflated = AABB.Combine(leafAABB, Nodes[left].aabb).SurfaceArea();
                    double branchArea = Nodes[left].aabb.SurfaceArea();
                    leftDescentCost = inflated - branchArea + inheritedCost;
                }

                // Cost for right descent
                double rightDescentCost;
                if (Nodes[right].IsLeaf())
                    rightDescentCost = AABB.Combine(leafAABB, Nodes[right].aabb).SurfaceArea() + inheritedCost;
                else
                {
                    double inflated = AABB.Combine(leafAABB, Nodes[right].aabb).SurfaceArea();
                    double branchArea = Nodes[right].aabb.SurfaceArea();
                    rightDescentCost = inflated - branchArea + inheritedCost;
                }

                // Determine traversal direction, or early out on a branch index
                if (branchCost < leftDescentCost && branchCost < rightDescentCost)
                    break;

                if (leftDescentCost < rightDescentCost)
                    searchIndex = left;

                else
                    searchIndex = right;
            }

            int sibling = searchIndex;

            // Create new parent
            int oldParent = Nodes[sibling].parent;
            int newParent = AllocateNode();
            Nodes[newParent].parent = oldParent;
            Nodes[newParent].userData = null;
            Nodes[newParent].aabb = AABB.Combine(leafAABB, Nodes[sibling].aabb);
            Nodes[newParent].height = Nodes[sibling].height + 1;

            // Sibling was root
            if (oldParent == Node.Null)
            {
                Nodes[newParent].left = sibling;
                Nodes[newParent].right = id;
                Nodes[sibling].parent = newParent;
                Nodes[id].parent = newParent;
                Root = newParent;
            }

            else
            {
                if (Nodes[oldParent].left == sibling)
                    Nodes[oldParent].left = newParent;

                else
                    Nodes[oldParent].right = newParent;

                Nodes[newParent].left = sibling;
                Nodes[newParent].right = id;
                Nodes[sibling].parent = newParent;
                Nodes[id].parent = newParent;
            }

            SyncHeirarchy(Nodes[id].parent);
        }

        void RemoveLeaf(int id)
        {
            if (id == Root)
            {
                Root = Node.Null;
                return;
            }

            // Setup parent, grandParent and sibling
            int parent = Nodes[id].parent;
            int grandParent = Nodes[parent].parent;
            int sibling;

            if (Nodes[parent].left == id)
                sibling = Nodes[parent].right;

            else
                sibling = Nodes[parent].left;

            // Remove parent and replace with sibling
            if (grandParent != Node.Null)
            {
                // Connect grandParent to sibling
                if (Nodes[grandParent].left == parent)
                    Nodes[grandParent].left = sibling;

                else
                    Nodes[grandParent].right = sibling;

                // Connect sibling to grandParent
                Nodes[sibling].parent = grandParent;
            }

            // Parent was root
            else
            {
                Root = sibling;
                Nodes[sibling].parent = Node.Null;
            }

            DeallocateNode(parent);
            SyncHeirarchy(grandParent);
        }

        void ValidateStructure(int index)
        {
            Node n = Nodes[index];

            int il = n.left;
            int ir = n.right;

            if (n.IsLeaf())
            {
                Assert(ir == Node.Null);
                Assert(n.height == 0);
                return;
            }

            Assert(il >= 0 && il < Capacity);
            Assert(ir >= 0 && ir < Capacity);
            Node l = Nodes[il];
            Node r = Nodes[ir];

            Assert(l.parent == index);
            Assert(r.parent == index);

            ValidateStructure(il);
            ValidateStructure(ir);

        }
        void RenderNode(Render render, int index)
        {
            Assert(index >= 0 && index < Capacity);

            Node n = Nodes[index];
            AABB b = n.aabb;

            render.SetPenPosition(b.min.x, b.max.y, b.min.z);

            render.Line(b.min.x, b.max.y, b.max.z);
            render.Line(b.max.x, b.max.y, b.max.z);
            render.Line(b.max.x, b.max.y, b.min.z);
            render.Line(b.min.x, b.max.y, b.min.z);

            render.SetPenPosition(b.min.x, b.min.y, b.min.z);

            render.Line(b.min.x, b.min.y, b.max.z);
            render.Line(b.max.x, b.min.y, b.max.z);
            render.Line(b.max.x, b.min.y, b.min.z);
            render.Line(b.min.x, b.min.y, b.min.z);

            render.SetPenPosition(b.min.x, b.min.y, b.min.z);
            render.Line(b.min.x, b.max.y, b.min.z);
            render.SetPenPosition(b.max.x, b.min.y, b.min.z);
            render.Line(b.max.x, b.max.y, b.min.z);
            render.SetPenPosition(b.max.x, b.min.y, b.max.z);
            render.Line(b.max.x, b.max.y, b.max.z);
            render.SetPenPosition(b.min.x, b.min.y, b.max.z);
            render.Line(b.min.x, b.max.y, b.max.z);

            if (!n.IsLeaf())
            {
                RenderNode(render, n.left);
                RenderNode(render, n.right);
            }
        }

        // Correct AABB hierarchy heights and AABBs starting at supplied
        // index traversing up the heirarchy
        void SyncHeirarchy(int index)
        {
            while (index != Node.Null)
            {
                index = Balance(index);

                int left = Nodes[index].left;
                int right = Nodes[index].right;

                Nodes[index].height = 1 + Math.Max(Nodes[left].height, Nodes[right].height);
                Nodes[index].aabb = AABB.Combine(Nodes[left].aabb, Nodes[right].aabb);

                index = Nodes[index].parent;
            }
        }

        // Insert nodes at a given index until Capacity into the free list
        void AddToFreeList(int index)
        {
            for (int i = index; i < Capacity - 1; ++i)
            {
                Nodes[i] = new Node();
                Nodes[i].next = i + 1;
                Nodes[i].height = Node.Null;
            }

            Nodes[Capacity - 1] = new Node();
            Nodes[Capacity - 1].next = Node.Null;
            Nodes[Capacity - 1].height = Node.Null;
            FreeList = index;
        }

        int Root;
        Node[] Nodes;
        int Count;    // Number of active nodes
        int Capacity; // Max capacity of nodes
        int FreeList;

        public static void FattenAABB(ref AABB aabb)
        {
            double k_fattener = 0.5;
            Vec3 v = new Vec3(k_fattener, k_fattener, k_fattener);

            aabb.min -= v;
            aabb.max += v;
        }


    }
}
