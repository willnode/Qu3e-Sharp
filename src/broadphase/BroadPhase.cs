//--------------------------------------------------------------------------------------------------
/**
    Qu3e Physics Engine - C# Version 1.01

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
    public struct ContactPair
    {
        public int A;
        public int B;
    }

    public interface ITreeCallback
    {
        bool TreeCallback(int id);
    }

    public class BroadPhase : ITreeCallback
    {
        public BroadPhase(ContactManager manager)
        {
            Manager = manager;

            PairBuffer = new List<ContactPair>();
            MoveBuffer = new List<int>();

        }

        public void InsertBox(Box shape, AABB aabb)
        {
            int id = Tree.Insert(aabb, shape);
            shape.broadPhaseIndex = id;
            BufferMove(id);
        }

        public void RemoveBox(Box shape)
        {
            Tree.Remove(shape.broadPhaseIndex);
        }

        // Generates the contact list. All previous contacts are returned to the allocator
        // before generation occurs.
        public void UpdatePairs()
        {
            PairBuffer.Clear();

            // Query the tree with all moving boxs
            for (int i = 0; i <  MoveBuffer.Count; ++i)
            {
                CurrentIndex = MoveBuffer[i];
                AABB aabb = Tree.GetFatAABB(CurrentIndex);

                // @TODO: Use a static and non-static tree and query one against the other.
                //        This will potentially prevent (gotta think about this more) time
                //        wasted with queries of static bodies against static bodies, and
                //        kinematic to kinematic.
                Tree.Query(this, aabb);
            }

            // Reset the move buffer
            MoveBuffer.Clear();

            // Sort pairs to expose duplicates
            PairBuffer.Sort(ContactPairSorter.Default);

            // Queue manifolds for solving
            {
                int i = 0;
                while (i < PairBuffer.Count)
                {
                    // Add contact to manager
                    ContactPair pair = PairBuffer[i];
                    Box A = (Box)Tree.GetUserData(pair.A);
                    Box B = (Box)Tree.GetUserData(pair.B);
                    Manager.AddContact(A, B);

                    ++i;

                    // Skip duplicate pairs by iterating i until we find a unique pair
                    while (i < PairBuffer.Count)
                    {
                        ContactPair potentialDup = PairBuffer[i];

                        if (pair.A != potentialDup.A || pair.B != potentialDup.B)
                            break;

                        ++i;
                    }
                }
            }

//            Tree.Validate();
        }

        public void Update(int id, AABB aabb)
        {
            if (Tree.Update(id, aabb))
                BufferMove(id);
        }

        public bool TestOverlap(int A, int B)
        {
            return AABB.AABBtoAABB(Tree.GetFatAABB(A), Tree.GetFatAABB(B));
        }

        ContactManager Manager;

        List<ContactPair> PairBuffer;

        List<int> MoveBuffer;

        internal DynamicAABBTree Tree = new DynamicAABBTree();
        int CurrentIndex;

        void BufferMove(int id)
        {
            MoveBuffer.Add(id);
        }

        public bool TreeCallback(int index)
        {
            // Cannot collide with self
            if (index == CurrentIndex)
                return true;

            

            int iA = Math.Min(index, CurrentIndex);
            int iB = Math.Max(index, CurrentIndex);


            PairBuffer.Add(new ContactPair() { A = iA, B = iB });

            return true;

        }

        //--------------------------------------------------------------------------------------------------

        public class ContactPairSorter : IComparer<ContactPair>
        {
            static ContactPairSorter m_default;
            public static ContactPairSorter Default
            {
                get
                {
                    return m_default ?? (m_default = new ContactPairSorter());
                }
            }

            public int Compare(ContactPair lhs, ContactPair rhs)
            {
                if (lhs.A == rhs.A)
                    return Comparer<int>.Default.Compare(lhs.B, rhs.B);
                else
                    return Comparer<int>.Default.Compare(lhs.A, rhs.A);
            }

        }



    }
}
