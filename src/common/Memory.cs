using System;
using System.Collections.Generic;
using static Qu3e.Settings;

namespace Qu3e
{

    // Instance Pooling Stack implementation
    internal class Stack<T> : List<T> where T : new()
    {
        public T Pop()
        {
            if (Count == 0)
                return new T();
            else
            {
                var r = this[Count - 1];
                RemoveAt(Count - 1);
                return r;
            }
        }

        public void Push(T t)
        {
            Add(t);
        }
    }

    // List Pooling
    internal static class ListPool<T> where T : new()
    {
        static Stack<List<T>> stack = new Stack<List<T>>();

        public static List<T> Pop ()
        {
            return stack.Pop();
        }

        public static void Push (List<T> t)
        {
            t.Clear();
            stack.Push(t);
        }
    }

    internal class ArrayPool<T> : List<T[]> 
    {
        int size;

        public ArrayPool(int size)
        {
            this.size = size;
        }

        public T[] Pop()
        {
            if (Count == 0)
                return new T[size];
            else
            {
                var r = this[Count - 1];
                RemoveAt(Count - 1);
                return r;
            }
        }

        public void Push(T[] t)
        {
            
            Add(t);
        }
    }

}
