using DocumentFormat.OpenXml.Office2013.Drawing.ChartStyle;
using DocumentFormat.OpenXml.Office2016.Drawing.Command;
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

            Graph graph = Graph.GraphWithXVertices(5);
            Graph graphEmpt = new Graph();
            Graph graphComplet = Graph.GenerateCompleteGraph(5);


            graph.printAdjacencyMatrix();
            graph.PrintAdjacencyList();

            Console.WriteLine("criação da aresta 1 --> 2: ");
            graph.AddEdgeWithNodeId(1,2);
            graph.AddEdgeWithNodeId(2,4);
            graph.PrintAdjacencyList();
            Console.WriteLine("ponderação da aresta 1 --> 2 com peso 5");
            graph.AddEdgeWeight(5, graph.GetNodeById(1), graph.GetNodeById(2));
            graph.printAdjacencyMatrix();
            Console.WriteLine("rotulação da aresta 1 --> 2 para aresta1-2: ");
            graph.AddEdgeLabel(graph.GetNodeById(1), graph.GetNodeById(2), "aresta1-2");
            Console.WriteLine("rotulação da aresta 4 --> 2 para aresta4-2: ");
            graph.AddEdgeLabel(graph.GetNodeById(4), graph.GetNodeById(2), "aresta4-2");
            Console.WriteLine("Rotulação do vertice 2: ");
            graph.AddNodeLabel(graph.GetNodeById(2), "dois");
            Console.Write("checagem de adjacencia entre os vertices 1 e 2: ");
            Console.WriteLine(graph.AreNodeAdjacency(graph.GetNodeById(1), graph.GetNodeById(2)));
            Console.Write("checagem de adjacencia entre os vertices 3 e 4: ");
            Console.WriteLine(graph.AreNodeAdjacency(graph.GetNodeById(3), graph.GetNodeById(4)));
            Console.Write("checagem de adjacencia das arestas  1 --> 2 e 3 --> 2: ");
            Console.WriteLine(graph.AreEdgesAdjacent("aresta1-2", "aresta4-2"));
            Console.Write("checagem da existencia da aresta 3 --> 2: ");
            Console.WriteLine(graph.ExistsEdge(graph.GetNodeById(3), graph.GetNodeById(2)));
            Console.Write("checagem da existencia da aresta 4 --> 2: ");
            Console.WriteLine(graph.ExistsEdge(graph.GetNodeById(4), graph.GetNodeById(2)));
            Console.Write("Qunatidade de vertices: ");
            Console.WriteLine(graph.CountNodes());
            Console.Write("Quantidade de arestas: ");
            Console.WriteLine(graph.CountEdges());
            Console.Write("checagem se o grafo 1 está vazio: ");
            Console.WriteLine(graph.IsEmpty());
            Console.Write("checagem se o grafo 1 é completo: ");
            Console.WriteLine(graph.IsComplete());
            Console.Write("checagem se o grafo 2 está vazio: ");
            Console.WriteLine(graphEmpt.IsEmpty());
            Console.Write("checagem se o grafo 2 é completo: ");
            Console.WriteLine(graphEmpt.IsComplete());
            Console.Write("checagem se o grafo 3 está vazio: ");
            Console.WriteLine(graphComplet.IsEmpty());
            Console.Write("checagem se o grafo 3 é completo: ");
            Console.WriteLine(graphComplet.IsComplete());
            graphComplet.printAdjacencyMatrix();

            //entrega 2
            Graph graphSend2 = Graph.GenerateConnectedGraph(10);
            Node node = new Node(100);
            graphSend2.AddNode(node);
            graphSend2.AddEdge(node, graphSend2.GetNodeById(1));
            Console.WriteLine("Verificando pontes com Naive: ");
            graphSend2.PrintBridgesNaive();
            Console.WriteLine("Verificando pontes com Tarjan");
            foreach(Edge vert in graphSend2.FindBridgesTarjanRec())
            {
                Console.WriteLine(vert.Origem.Id + " --> " + vert.Destino.Id);
            }
            graphSend2.removeEdge(graphSend2.GetNodeById(100), graphSend2.GetNodeById(1));
            graphSend2.removeNode(graphSend2.GetNodeById(100));

            List<Node> nodes = graphSend2.FleuryNaive();
            List<Node> nodest = graphSend2.FleuryTarjan();
            Console.WriteLine("caminho euleriano Naive: ");
            foreach (Node n in nodes)
            {
                Console.Write(n.Id + " ");
            }
            Console.WriteLine();
            Console.WriteLine("caminho euleriano Tarjan: ");
            foreach (Node n in nodest)
            {
                Console.Write(n.Id + " ");
            }


            /*
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
             Console.WriteLine("removendo vétice: ");
             graph.removeNode(teste);
             graph.printMatrix();
             graph.printGraph();
             Console.WriteLine();
             Console.WriteLine("checando adjacencia entre os vertices a e b: " + graph.AreNodeAdjacency(node1, node2));
             graph.AddEdgeLabel(node1, node2, "label1");
             graph.AddEdgeLabel(node2, node3, "label2");
             graph.AddEdgeLabel(node4, node2, "label3");
             Console.WriteLine("checando adjacencia entre as arestas 1 e 2: " + graph.AreEdgesAdjacent("label1", "label2"));
             Console.WriteLine("checando adjacencia entre as arestas 1 e 3: " + graph.AreEdgesAdjacent("label1", "label3"));
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
            graph.AddEdge(node1, node2, 1);
            graph.AddEdge(node2, node3, 1);
            graph.AddEdge(node1, node3, 2);
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
            graph.AddEdgeLabel(node1, node3, "label1");
            Console.WriteLine("Checando adjacência entre as arestas 1 e 2: " + graph.AreEdgesAdjacent("label1", "label2")); // Deve imprimir False
            Console.WriteLine("Checando se existe a aresta 'A'->'C': " + graph.ExistsEdge(node1, node3)); // Deve imprimir True

            // Testando métodos de conectividade
            Console.WriteLine("Grafo está conectado: " + graph.isConnected()); // Deve imprimir False
            graph.AddEdge(node1, node3);
            Console.WriteLine("Grafo está conectado: " + graph.isConnected()); // Deve imprimir True

            // Testando métodos de verificação de grafos
            Console.WriteLine("Grafo é completo: " + graph.IsComplete()); // Deve imprimir True

            // Testando métodos de identificação de pontes
            Console.WriteLine("Pontes encontradas pelo método Naive:");
            graph.PrintBridgesNaive();

            Console.WriteLine("Pontes encontradas pelo método de Tarjan:");
            graph.PrintBridgesTarjan();



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

            /*Stopwatch sw = new Stopwatch();
            sw.Start();
            Graph graphtn = Graph.GenerateConnectedGraph(100);
            sw.Stop();

            graph.PrintAdjacencyList();
            //graph.printGraph(); // Para visualizar o grafo criado
            //graph.printMatrix();
            //graph.printedges();
            //graph.GetCSV();
            //Console.WriteLine(graph.CountEdges());


            Stopwatch sw2 = new Stopwatch();
            //graph.GetCSV();


            sw2.Start();
            List<Node> eulerianCycle = graphtn.FleuryTarjan();
            sw2.Stop();
            Console.WriteLine("Tempo de criação do grafo com 1000 vértices: " + sw.Elapsed);
            Console.WriteLine("Tempo de execução do Fleury com Tarjan: " + sw2.Elapsed);


            Console.WriteLine("caminho euleriano");
            if (eulerianCycle != null)
            {
                foreach (Node node in eulerianCycle)
                {
                    Console.Write(node.Id + " ");
                }
            }*/

        }

    }
}
