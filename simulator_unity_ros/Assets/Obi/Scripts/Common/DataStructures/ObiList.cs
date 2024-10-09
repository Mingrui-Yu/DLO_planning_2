using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi
{
	/**
	 * Custom IList implementation that allows access to the underlying raw array, for efficient C++ interop. Also
	 * includes some auxiliar methods that make it easier and faster to send data back and forth between C# and C++, as
     * well as deal with accesing the contents of the list directly without a copy.
	 */
	public class ObiList<T> : IList<T>
	{
		private T[] data = new T[16];
		private int count = 0;
	
	    #region Implementation of IEnumerable
	
	    public IEnumerator<T> GetEnumerator()
	    {
			for (int i = 0; i < count;++i)
				yield return data[i];
	    }
	
	    IEnumerator IEnumerable.GetEnumerator()
	    {
	        return GetEnumerator();
	    }
	
	    #endregion
	
	    #region Implementation of ICollection<T>
	
	    public void Add(T item)
	    {
            EnsureCapacity(count + 1);
            data[count++] = item;
	    }
	
	    public void Clear()
	    {
			count = 0;
	    }
	
	    public bool Contains(T item)
	    {
			for (int i = 0; i < count; ++i)
				if (data[i].Equals(item)) 
					return true;
			return false;
	    }
	
	    public void CopyTo(T[] array, int arrayIndex)
	    {
			if (array == null)
				throw new ArgumentNullException();

			if (array.Length-arrayIndex < count)
				throw new ArgumentException();

			Array.Copy(data,0,array,arrayIndex,count);
	    }
	
	    public bool Remove(T item)
	    {
			bool found = false;
			for (int i = 0; i < count; ++i)
			{
				// Look for the element, and mark it as found.
				if (!found && data[i].Equals(item))
					found = true;
				
				//If we found the element and are not at the last element, displace the element 1 position backwards.
				if (found && i < count-1)
				{
					data[i] = data[i+1];
				}
			}	

			// If we found and removed the element, reduce the element count by 1.
			if (found)
				count--;

			return found;		
	    }
	
	    public int Count
	    {
	        get { return count; }
	    }
	
	    public bool IsReadOnly
	    {
			get { return false; }
	    }
	
	    #endregion
	
	    #region Implementation of IList<T>
	
	    public int IndexOf(T item)
	    {
			return Array.IndexOf(data,item);
	    }
	
	    public void Insert(int index, T item)
	    {
			if (index < 0 || index > count)
				throw new ArgumentOutOfRangeException();

			EnsureCapacity(++count);

	        for (int i = count-1; i > index; ++i)
			{
				data[i] = data[i-1];
			}

			data[index] = item;
	    }
	
	    public void RemoveAt(int index)
	    {
			for (int i = index; i < count; ++i)
			{
				if (i < count-1)
				{
					data[i] = data[i+1];
				}
			}	

			count--;
	    }
	
	    public T this[int index]
	    {
	        get { return data[index]; }
	        set { data[index] = value; }
	    }
	
	    #endregion
	
	    #region Added stuff
	
		public T[] Data{
			get{return data;}
		}

		/**
		 * Ensures a minimal capacity of count elements, then sets the new count. Useful when passing the backing array to C++
		 * for being filled with new data.
		 */
		public void SetCount(int count){
			EnsureCapacity(count);
			this.count = count;
		}

		public void EnsureCapacity(int capacity){	
			if (capacity >= data.Length)
				Array.Resize(ref data,capacity*2);
		}
	
	    #endregion
	}
}

