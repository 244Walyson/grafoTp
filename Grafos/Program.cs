using DocumentFormat.OpenXml.Office2013.Drawing.ChartStyle;
using DocumentFormat.OpenXml.Spreadsheet;
using System.Diagnostics;

namespace Grafos
{
    internal class Program
    {
        static void Main(string[] args)
        {
            /*Node node1 = new Node("a");
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
            graph.addEdge(node1, node2, 1);
            graph.addEdge(node2, node3, 1);
            graph.addEdge(node3, node4, 1);
            graph.addEdge(node4, node1, 1);
            graph.addEdge(node1, node3, 1);
            graph.addEdge(node2, node4, 1);
            // Imprimindo o grafo
            Console.WriteLine("Grafo:");
            graph.printGraph();
            Console.WriteLine("-----------------------------------------");
            graph.printMatrix();

            Console.WriteLine("testes ---------------------------------------------");

            Console.WriteLine("adicionando aresta: ");
            graph.addEdge(teste, node1, 5);
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
            Console.WriteLine("checando se o grafo esta vazio: " + graph.AreEmptyGraph());
            Console.WriteLine("checando se o grafo 2 esta vazio: " + graph2.AreEmptyGraph());
            Console.WriteLine("cheacando se o grafo e completo: "+ graph.AreCompletGraph());
            Console.WriteLine("cheacando se o grafo 2 e completo: "+ graph2.AreCompletGraph());



            Console.WriteLine("isconnected: " + graph.isConnected());
            graph.AddNode(teste);
            Console.WriteLine("isconnected: " + graph.isConnected());

            graph.addEdge(node1, teste);
            graph.printGraph();
            Console.Write("bridges: " );
            graph.PrintBridgesNaive();
            Console.WriteLine("usando tarjam: ");
            graph.printBridgestarjam();*/

            Graph graph = Graph.GenerateRandomGraph(10000,60000);
            //graph.PrintBridgesNaive();
            //graph.GenerateXlsx();
            graph.GetCSV();



            /*using (var workbook = new XLWorkbook())
            {

                var worksheet = workbook.Worksheets.Add("grafo");
                worksheet.Cell("A1").Value = "testando";
                worksheet.Cell("A2").Value = "testando";
                worksheet.Cell("A3").Value = "testando";
                worksheet.Cell("A4").Value = "testando";
                worksheet.Cell("A5").Value = "testando";

                workbook.SaveAs(@"C:\Users\walys\Desktop\teste.xlsx");
            }

            Process.Start(new ProcessStartInfo(@"C:\Users\walys\Desktop\grafo.xlsx") { UseShellExecute = true });*/
        }
    }
}
