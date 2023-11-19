using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Grafos
{
    public class Node
    {
        public Object Id { get; set; }
        public Object Label { get; set; }
        public Node(Object id)
        {
            Id = id;
        }

        public void AddLabel(Object label)
        {
            Label = label;
        }
    }
}
