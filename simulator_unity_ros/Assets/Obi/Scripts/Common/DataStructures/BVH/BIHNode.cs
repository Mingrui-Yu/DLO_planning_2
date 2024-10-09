using System;

namespace Obi
{
    public struct BIHNode
    {
        public int firstChild;     /**< index of the first child node. The second one is right after the first.*/
        public int start;          /**< index of the first element in this node.*/
        public int count;          /**< amount of elements in this node.*/

        public int axis;           /**< axis of the split plane (0,1,2 = x,y,z)*/
        public float min;          /**< minimum split plane*/
        public float max;          /**< maximum split plane*/

        public BIHNode(int start, int count)
        {
            firstChild = -1;
            this.start = start;
            this.count = count;
            axis = 0;
            min = float.MinValue;
            max = float.MaxValue;
        }
    }
}
