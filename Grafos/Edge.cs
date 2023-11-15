using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Grafos
{
    public class Edge
    {
        public Node Origem { get; set; }
        public Node Destino { get; set; }
        public int Peso { get; set; }
        public Object Label { get; set; }

        public Edge(Node origem, Node destino, int peso = 1, object label = null)
        {
            Origem = origem;
            Destino = destino;
            Peso = peso;
            Label = label;
        }

        public void addLabel(Object label)
        {
            label = label;
        }
    }
}
