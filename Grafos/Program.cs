using DocumentFormat.OpenXml.Office2013.Drawing.ChartStyle;
using DocumentFormat.OpenXml.Spreadsheet;
using System.Diagnostics;
using System.Threading;

namespace Grafos
{
    internal class Program
    {

        [LoaderOptimization(LoaderOptimization.MultiDomain)]
        static void Main(string[] args)
        {
            /* Node node1 = new Node("a");
             Node node2 = new Node("b");
             Node node3 = new Node("c");
             Node node4 = new Node("d");
             Node teste = new Node("t");

             // Criando uma instância de um grafo não direcionado
             Graph graph = new Graph();
             Graph graph2 = new Graph();

             // Adicionando nós ao grafo
             graph.AddNode(node1);
             graph.AddNode(node2);
             graph.AddNode(node3);
             graph.AddNode(node4);
             //node para teste
             graph.AddNode(teste);

             // Adicionando arestas ao grafo
             graph.AddEdge(node2, node3, 1);
             graph.AddEdge(node1, node2, 1);
             graph.AddEdge(node3, node4, 1);
             graph.AddEdge(node4, node1, 1);
             graph.AddEdge(node1, node3, 1);
             graph.AddEdge(node2, node4, 1);
             // Imprimindo o grafo
             Console.WriteLine("Grafo:");
             graph.printGraph();
             Console.WriteLine("-----------------------------------------");
             graph.printMatrix();

             Console.WriteLine("testes ---------------------------------------------");

             Console.WriteLine("adicionando aresta: ");
             graph.AddEdge(teste, node1, 5);
             graph.printMatrix();
             graph.printGraph();
             Console.WriteLine("removento aresta: ");
             graph.removeEdge(teste, node1);
             graph.printMatrix();
             graph.printGraph();
             Console.WriteLine("removendo aresta: ");
             graph.removeNode(teste);
             graph.printMatrix();
             graph.printGraph();
             Console.WriteLine();
             Console.WriteLine("checando adjacencia entre os vertices a e b: " + graph.AreNodeAdjacency(node1, node2));
             graph.addEdgeLabel(node1, node2, "label1");
             graph.addEdgeLabel(node2, node3, "label2");
             graph.addEdgeLabel(node4, node2, "label3");
             Console.WriteLine("checando adjacencia entre as arestas 1 e 2: " + graph.AreEdgesAdjacent("label1", "label2"));
             Console.WriteLine("checando adjacencia entre as arestas 1 e 2: " + graph.AreEdgesAdjacent("label1", "label3"));
             Console.WriteLine("checando se existe a aresta 'a'->'b': " + graph.ExistsEdge(node1, node2));
             Console.WriteLine("checando a quantidade de vertices e arestas: ");
             Console.WriteLine("vertices: " + graph.CountNodes());
             Console.WriteLine("Arestas: " + graph.CountEdges());
             Console.WriteLine("checando se o grafo esta vazio: " + graph.IsEmpty());
             Console.WriteLine("checando se o grafo 2 esta vazio: " + graph2.IsEmpty());
             Console.WriteLine("cheacando se o grafo e completo: "+ graph.IsEmpty());
             Console.WriteLine("cheacando se o grafo 2 e completo: "+ graph2.IsEmpty());



             Console.WriteLine("isconnected: " + graph.isConnected());
             graph.AddNode(teste);
             Console.WriteLine("isconnected: " + graph.isConnected());

             graph.AddEdge(node1, teste);
             graph.printGraph();
             Console.Write("bridges: " );
             graph.PrintBridgesNaive();
             Console.WriteLine("usando tarjam: ");
             graph.PrintBridgesTarjan();*/

            /*Graph graph = new Graph();
            Node node1 = new Node("A");
            Node node2 = new Node("B");
            Node node3 = new Node("C");
            graph.AddNode(node1);
            graph.AddNode(node2);
            graph.AddNode(node3);

            // Testando adição de arestas e impressão do grafo
            graph.addEdge(node1, node2, 1);
            graph.addEdge(node2, node3, 1);
            graph.addEdge(node1, node3, 2);
            Console.WriteLine("Grafo:");
            graph.printGraph();

            // Testando métodos de contagem
            Console.WriteLine("Número de vértices: " + graph.CountNodes()); // Deve imprimir 3
            Console.WriteLine("Número de arestas: " + graph.CountEdges());  // Deve imprimir 2

            // Testando remoção de arestas
            Console.WriteLine("Removendo aresta entre A e B:");
            graph.removeEdge(node1, node2);
            graph.printGraph();

            // Testando remoção de nó
            Console.WriteLine("Removendo nó B:");
            graph.removeNode(node2);
            graph.printGraph();

            // Testando adjacência de nós e arestas
            Console.WriteLine("Checando adjacência entre A e C: " + graph.AreNodeAdjacency(node1, node3)); // Deve imprimir False
            graph.addEdgeLabel(node1, node3, "label1");
            Console.WriteLine("Checando adjacência entre as arestas 1 e 2: " + graph.AreEdgesAdjacent("label1", "label2")); // Deve imprimir False
            Console.WriteLine("Checando se existe a aresta 'A'->'C': " + graph.ExistsEdge(node1, node3)); // Deve imprimir True

            // Testando métodos de conectividade
            Console.WriteLine("Grafo está conectado: " + graph.isConnected()); // Deve imprimir False
            graph.addEdge(node1, node3);
            Console.WriteLine("Grafo está conectado: " + graph.isConnected()); // Deve imprimir True

            // Testando métodos de verificação de grafos
            Console.WriteLine("Grafo é completo: " + graph.AreCompletGraph()); // Deve imprimir True

            // Testando métodos de identificação de pontes
            Console.WriteLine("Pontes encontradas pelo método Naive:");
            graph.PrintBridgesNaive();

            Console.WriteLine("Pontes encontradas pelo método de Tarjan:");
            graph.PrintBridgesTarjan();*/



            /*Graph graph = Graph.GenerateCompleteGraph(5);
           Console.WriteLine(graph.IsComplete());
           graph.printMatrix();

           Console.WriteLine("naive------------------");
           //graph.PrintBridgesNaive();
           Console.WriteLine("tarjan----------------");
           graph.PrintBridgesTarjan();

           List<Node> eulerianCycle = graph.Fleury();

           foreach (Node node in eulerianCycle)
           {
               Console.WriteLine(node.Id);
           }*/

            Graph graph = Graph.GenerateConnectedGraph(10);

            /*Node a = new Node("A");
            Node b = new Node("B");
            Node c = new Node("C");
            Node d = new Node("D");

            graph.AddNode(a);
            graph.AddNode(b);
            graph.AddNode(c);
            graph.AddNode(d);

            graph.AddEdge(a, b);
            graph.AddEdge(b, c);
            graph.AddEdge(c, d);
            graph.AddEdge(d, a);*/

            //graph.printGraph(); // Para visualizar o grafo criado
            //graph.printMatrix();
            graph.printedges();
            //graph.GetCSV();
            //Console.WriteLine(graph.CountEdges());

            List<Node> eulerianCycle = graph.Fleury();

            Console.WriteLine("caminho euleriano");
            if (eulerianCycle != null)
            {
                foreach (Node node in eulerianCycle)
                {
                    Console.Write(node.Id + " ");
                }
            }
            
        }
    }
}
