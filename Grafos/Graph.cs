using System.Diagnostics;
using ClosedXML.Excel;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using DocumentFormat.OpenXml.Spreadsheet;

namespace Grafos
{
    public class Graph
    {
        private List<Node> nodes;
        private List<Edge> edges;
        public bool isDirected { get; set; }
        public int[,] adjacencyMatrix { get; set; }


        public Graph(bool directed = false)
        {
            nodes = new List<Node>();
            edges = new List<Edge>();
            isDirected = directed;
        }

        public void AddNode(Node node)
        {
            nodes.Add(node);
        }

        public void AddEdge(Node origem, Node destino, int peso = 1)
        {
            Edge newEdge = new Edge(origem, destino, peso);
            edges.Add(newEdge);
            if (!isDirected)
            {
                Edge inverseEdge = new Edge(destino, origem, peso);
                edges.Add(inverseEdge);
            }
        }

        // Representação usando matrix de adjacencia 
        public void getAdjacencyMatriz()
        {
            adjacencyMatrix = new int[nodes.Count, nodes.Count];

            Console.WriteLine(nodes.Count);
            for (int i = 0; i < nodes.Count; i++)
            {
                for (int j = 0; j < nodes.Count; j++)
                {
                    adjacencyMatrix[i, j] = 0;
                }
            }

            foreach (var edge in edges)
            {
                int i = nodes.IndexOf(edge.Origem);
                int j = nodes.IndexOf(edge.Destino);
                adjacencyMatrix[i, j] = edge.Peso;
            }
        }

        public void printAdjacencyMatrix()
        {
            getAdjacencyMatriz();
            for (int k = -1; k < nodes.Count; k++)
            {
                if (k == -1)
                {
                    Console.Write("  | ");
                }
                else
                {
                    Console.Write(nodes[k].Id + " | ");
                }
            }
            Console.WriteLine();
            for (int i = 0; i < nodes.Count; i++)
            {
                Console.Write(nodes[i].Id + " | ");
                for (int j = 0; j < nodes.Count; j++)
                {
                    Console.Write(adjacencyMatrix[i, j] + " | ");
                }
                Console.WriteLine();
            }
        }

        // Adição de Rotulo as arestas
        public void addEdgeLabel(Node node1, Node node2, Object label)
        {
            Edge edge = edges.Find(edge => (edge.Origem == node1 && edge.Destino == node2) || (edge.Origem == node2 && edge.Destino == node1));
            edge.addLabel(label);
        }

        public void removeEdge(Node source, Node destiny)
        {
            Edge edgeToRemove = edges.Find(edge => edge.Origem == source && edge.Destino == destiny);
            Edge edgeNotDirected = edges.Find(edge => edge.Destino == source && edge.Origem == destiny);

            if (edgeToRemove != null)
            {
                edges.Remove(edgeToRemove);
                if (!isDirected)
                {
                    edges.Remove(edgeNotDirected);
                }
            }
        }

        // Remoção de Vertices
        public void removeNode(Node node)
        {
            nodes.Remove(node);
        }

        public bool AreNodeAdjacency(Node source, Node destiny)
        {
            Edge edgeToCheck = edges.Find(edge => edge.Origem == source && edge.Destino == destiny);

            if (edgeToCheck != null)
            {
                return true;
            }
            return false;
        }

        // Verifica se as arestas são adjacentes
        public bool AreEdgesAdjacent(Object a, Object b)
        {
            Edge edge1 = edges.Find(edge => edge.Label == a);
            Edge edge2 = edges.Find(edge => edge.Label == b);

            if (edge1 != null && edge2 != null)
            {
                return edge1.Origem == edge2.Origem || edge1.Destino == edge2.Destino ||
                   edge1.Origem == edge2.Destino || edge1.Destino == edge2.Origem;
            }
            return false;
        }

        // Verifica se existe a aresta
        public bool ExistsEdge(Node source, Node destiny)
        {
            return edges.Any(edge => (edge.Origem == source && edge.Destino == destiny) || (edge.Origem == destiny && edge.Destino == source));
        }

        // Quantidade de arestas
        public int CountEdges()
        {
            return edges.Count;
        }

        // Quantidade de vertices
        public int CountNodes()
        {
            return nodes.Count;
        }

        // Verifica se o grafo e completo
        public bool IsComplete()
        {
            int countNodes = CountNodes();
            int countEdges = CountEdges() / 2;
            int maxEdges = (countNodes * (countNodes - 1)) / 2;

            if (IsEmpty() || maxEdges < countEdges)
            {
                return false;
            }

            for (int i = 0; i < nodes.Count - 1; i++)
            {
                for (int j = i + 1; j < nodes.Count; j++)
                {
                    if (!AreNodeAdjacency(nodes[i], nodes[j]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        // Verifica se o grafo esta vazio
        public bool IsEmpty()
        {
            return edges.Count == 0;
        }

        // verifica se o grafo esta conectado *auxilia no metodo de achar pontes por naive*
        public bool isConnected()
        {
            if (nodes.Count == 0)
            {
                return true;
            }

            HashSet<Node> visited = new HashSet<Node>();
            Queue<Node> queue = new Queue<Node>();

            Node startNode = nodes[0];
            queue.Enqueue(startNode);
            visited.Add(startNode);

            while (queue.Count > 0)
            {
                Node currentNode = queue.Dequeue();
                List<Edge> adjacentEdges = GetEdges(currentNode);

                foreach (Edge edge in adjacentEdges)
                {
                    Node neigbhor = edge.Destino;
                    if (!visited.Contains(neigbhor))
                    {
                        queue.Enqueue(neigbhor);
                        visited.Add(neigbhor);
                    }
                }
            }

            return visited.Count == nodes.Count;

        }

        // Busca de pontes usando naive
        public List<Edge> FindBridgeNaive()
        {

            List<Edge> bridges = new List<Edge>();
            List<Edge> edgesCopy = new List<Edge>();

            foreach (var edge in edges)
            {
                bool isDuplicate = edgesCopy.Any(b => (b.Origem == edge.Destino && b.Destino == edge.Origem));

                if (!isDuplicate)
                {
                    edgesCopy.Add(edge);
                }
            }
            foreach (var edge in edgesCopy)
            {
                removeEdge(edge.Origem, edge.Destino);
                if (!isConnected())
                {
                    bridges.Add(edge);
                }
                AddEdge(edge.Origem, edge.Destino, edge.Peso);
            }
            return bridges;
        }

        // Busca um vertice por id
        public Node GetNodeById(Object id)
        {
            return nodes.Find(node => node.Id.Equals(id));
        }

        // Busca as arestas adjacentes a um vertice
        public List<Edge> GetEdges(Node node)
        {
            return edges.FindAll(edge => edge.Origem == node || edge.Destino == node);
        }


        // busca pontes usando tarjan
        private void TarjanBridgesMult(List<Edge> bridges)
        {
            int[] disc = new int[nodes.Count];
            int[] low = new int[nodes.Count];
            bool[] visited = new bool[nodes.Count];
            int time = 0;

            Stack<int> stack = new Stack<int>();

            object lockObj = new object();

            Parallel.ForEach(nodes, node =>
            {
                int u = nodes.IndexOf(node);
                if (!visited[u])
                {
                    stack.Push(u);
                    int parent = -1;
                    while (stack.Count > 0)
                    {
                        int currentU;
                        lock (lockObj)
                        {
                            if (stack.Count > 0)
                            {
                                currentU = stack.Peek();
                                stack.Pop();
                            }
                            else
                            {
                                break;
                            }
                        }

                        if (!visited[currentU])
                        {
                            visited[currentU] = true;
                            disc[currentU] = low[currentU] = Interlocked.Increment(ref time);

                            foreach (var edge in GetEdges(nodes[currentU]))
                            {
                                int v = nodes.IndexOf(edge.Destino);
                                if (v != -1 && !visited[v])
                                {
                                    stack.Push(v);
                                    parent = currentU;
                                }
                                else if (v != -1 && v != parent)
                                {
                                    low[currentU] = Math.Min(low[currentU], disc[v]);
                                }
                            }
                        }
                        else
                        {
                            foreach (var edge in GetEdges(nodes[currentU]))
                            {
                                int v = nodes.IndexOf(edge.Destino);
                                if (v != -1 && v != parent)
                                {
                                    low[currentU] = Math.Min(low[currentU], low[v]);
                                    if (low[v] > disc[currentU])
                                    {
                                        lock (lockObj)
                                        {
                                            bridges.Add(edge);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            });
        }
        public List<Edge> FindBridgesTarjan()
        {
            List<Edge> bridges = new List<Edge>();

            TarjanBridgesMult(bridges);

            return bridges;
        }


        //gera um grafo aleatorio
        public static Graph GenerateRandomGraph(int nodes, int edges, int numThreads)
        {
            Graph graph = new Graph();
            object lockObj = new object();
            ThreadLocal<Random> randomGenerator = new ThreadLocal<Random>(() => new Random());

            Parallel.For(0, numThreads, i =>
            {
                for (int j = 0; j < nodes / numThreads; j++)
                {
                    Node node = new Node(graph.nodes.Count + 1);
                    lock (lockObj)
                    {
                        graph.AddNode(node);
                    }
                }

                for (int k = 0; k < edges / numThreads; k++)
                {
                    int randomDestinoIndex = randomGenerator.Value.Next(graph.CountNodes());
                    int randomOrigemIndex = randomGenerator.Value.Next(graph.CountNodes());

                    Node destino = graph.GetNodeById(randomDestinoIndex);
                    Node origem = graph.GetNodeById(randomOrigemIndex);

                    if (destino != null && origem != null)
                    {
                        int weight = randomGenerator.Value.Next(10000);
                        lock (lockObj)
                        {
                            graph.AddEdge(origem, destino, weight);
                        }
                    }
                }
            });

            return graph;
        }

        // gera um grafo completo
        public static Graph GenerateCompleteGraph(int numNodes)
        {
            Graph graph = new Graph();
            for (int i = 1; i <= numNodes; i++)
            {
                graph.AddNode(new Node(i));
            }

            foreach (var node in graph.nodes)
            {
                foreach (var otherNode in graph.nodes)
                {
                    if (node != otherNode)
                    {
                        graph.AddEdge(node, otherNode);
                    }
                }
            }
            return graph;
        }


        //-----------------------------------------PRINT METHODS-------------------------------------------------

        public void printnodes()
        {
            foreach (Node n in nodes)
            {
                Console.WriteLine(n.Id);
            }
        }

        public void printGraph()
        {
            foreach (var node in nodes)
            {
                Console.Write(node.Id + " --> ");
                foreach (var edge in edges)
                {
                    if (edge.Origem == node)
                    {
                        Console.Write($"{edge.Destino.Id} (Peso: {edge.Peso}) -> ");
                    }
                }
                Console.WriteLine();
            }
        }

        public void printMatrix()
        {
            getAdjacencyMatriz();
            for (int k = -1; k < nodes.Count; k++)
            {
                if (k == -1)
                {
                    Console.Write("  | ");
                }
                else
                {
                    Console.Write(nodes[k].Id + " | ");
                }
            }
            Console.WriteLine();
            for (int i = 0; i < nodes.Count; i++)
            {
                Console.Write(nodes[i].Id + " | ");
                for (int j = 0; j < nodes.Count; j++)
                {
                    Console.Write(adjacencyMatrix[i, j] + " | ");
                }
                Console.WriteLine();
            }
        }

        public void PrintBridgesNaive()
        {
            if (isConnected())
            {
                List<Edge> bridges = FindBridgeNaive();
                foreach (var edge in bridges)
                {
                    Console.WriteLine(edge.Origem.Id + " --> " + edge.Destino.Id);
                }
            }
            else
            {
                Console.WriteLine("grafo desconexo não e possivel achar pontes");
            }
        }

        public void PrintBridgesTarjan()
        {
            List<Edge> bridges = FindBridgesTarjan();

            foreach (Edge e in bridges)
            {
                Console.WriteLine(e.Origem.Id + " --> " + e.Destino.Id);
            }
        }


        //------------------------------------------GEPHI---------------------------------------------------
        public void GenerateXlsx()
        {
            getAdjacencyMatriz();
            using (var workbook = new XLWorkbook())
            {
                var worksheet = workbook.Worksheets.Add("grafo1");

                for (int i = 0; i < nodes.Count; i++)
                {
                    worksheet.Cell(1, i + 2).Value = nodes[i].Id.ToString();
                    worksheet.Cell(i + 2, 1).Value = nodes[i].Id.ToString();

                    for (int j = 0; j < nodes.Count; j++)
                    {
                        worksheet.Cell(i + 2, j + 2).Value = adjacencyMatrix[i, j];
                    }
                }
                workbook.SaveAs(@"C:\Users\walys\Desktop\grafo.xlsx");
            }
            //Process.Start(new ProcessStartInfo(@"C:\Users\walys\Desktop\grafo.xlsx") { UseShellExecute = true });
        }

        public void GetCSV()
        {
            using (StreamWriter writer = new StreamWriter(@"C:\Users\walys\Desktop\grafo.csv"))
            {
                foreach (Node node in nodes)
                {
                    Console.WriteLine(node.Id);
                    writer.Write(node.Id.ToString());
                    foreach (Edge edge in edges)
                    {
                        if (edge.Origem == node)
                        {
                            writer.Write("; " + edge.Destino.Id);
                        }
                    }
                    writer.WriteLine();
                }
            }

            string caminhoDoCSV = @"C:\Users\walys\Desktop\grafo.csv";
            string args = $"--import-file \"{caminhoDoCSV}\"";
            Process.Start(@"C:\Program Files\Gephi-0.10.1\bin\gephi64.exe");
        }

        public List<Node> Fleury()
        {
            if (!isConnected() || CountOddNodes() > 2)
            {
                Console.WriteLine("não existe caminho euleriano possiivel");
                return null;
            }

            Node startNode = nodes.FirstOrDefault(node => GetEdges(node).Count % 2 != 0);

            startNode = nodes.FirstOrDefault();

            List<Node> euleryaCicle = new List<Node>();

            return euleryaCicle;

        }

        public void FleuryRecursive(Node currentNode, List<Node> eulerianCycle)
        {
            List<Edge> edgesCopy = new List<Edge>(edges);

            foreach (Edge edge in edgesCopy)
            {
                Node nextNode = edge.Destino;
                if (IsValidNextEdge(currentNode, nextNode))
                {
                    removeEdge(currentNode, nextNode);
                    eulerianCycle.Add(nextNode);
                    FleuryRecursive(nextNode, eulerianCycle);
                }
            }
        }
        public bool IsValidNextEdge(Node current, Node nextNode)
        {
            if (GetEdges(current).Count == 1)
            {
                return true;
            }
            if (IsBridge(current, nextNode))
            {
                return false;
            }

            Graph tempGraph = new Graph(isDirected);
            tempGraph.nodes.AddRange(nodes);
            tempGraph.edges.AddRange(edges);
            tempGraph.removeEdge(current, nextNode);

            if (!tempGraph.isConnected())
            {
                return false;
            }

            return true;
        }

        public bool IsBridge(Node current, Node nextNode)
        {
            List<Edge> bridges = FindBridgesTarjan();

            return bridges.Any(edge => (edge.Origem == current && edge.Destino == nextNode) || (edge.Origem == nextNode && edge.Destino == current));
        }


        public int CountOddNodes()
        {
            int count = 0;

            foreach (Node node in nodes)
            {
                int edgeCount = edges.Count(edge => edge.Origem == node || edge.Destino == node);

                if (!isDirected)
                {
                    edgeCount = edgeCount / 2;
                }

                if (edgeCount % 2 != 0)
                {
                    count++;
                }
            }

            return count;
        }


    }
}
