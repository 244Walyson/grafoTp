using System.Diagnostics;
using ClosedXML.Excel;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using DocumentFormat.OpenXml.Spreadsheet;
using DocumentFormat.OpenXml.EMMA;
using System.Xml.Linq;

namespace Grafos
{
    public class Graph
    {
        private List<Node> nodes;
        private List<Edge> edges;
        private List<Edge> uniqueEdges;
        private Dictionary<Node, List<Node>> adjacencyList;
        public bool isDirected { get; set; }
        public int[,] adjacencyMatrix { get; set; }


        public Graph(bool directed = false)
        {
            nodes = new List<Node>();
            edges = new List<Edge>();
            adjacencyList = new Dictionary<Node, List<Node>>();
            uniqueEdges = new List<Edge>();
            isDirected = directed;
        }
        public void AddNode(Node node)
        {
            nodes.Add(node);
            adjacencyList.Add(node, new List<Node>());
        }

        public void AddEdge(Node origem, Node destino, int peso = 1)
        {
            Edge newEdge = new Edge(origem, destino, peso);
            edges.Add(newEdge);
            uniqueEdges.Add(newEdge);
            adjacencyList[origem].Add(destino);

            if (!isDirected)
            {
                Edge inverseEdge = new Edge(destino, origem, peso);
                edges.Add(inverseEdge);
                adjacencyList[destino].Add(origem);
            }
        }

        public Dictionary<Node, List<Node>> GetAdjacencyList()
        {
            return adjacencyList;
        }


        public void PrintAdjacencyList()
        {
            foreach (var entry in adjacencyList)
            {
                Node node = entry.Key;
                List<Node> neighbors = entry.Value;

                Console.Write($"{node.Id}: ");

                foreach (var neighbor in neighbors)
                {
                    Console.Write($"{neighbor.Id} ");
                }

                Console.WriteLine();
            }
        }

        // Representação usando matrix de adjacencia 
        public void getAdjacencyMatriz()
        {
            adjacencyMatrix = new int[nodes.Count, nodes.Count];

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
        public void AddEdgeLabel(Node node1, Node node2, Object label)
        {
            Edge edge = edges.Find(edge => (edge.Origem == node1 && edge.Destino == node2) || (edge.Origem == node2 && edge.Destino == node1));
            edge.addLabel(label);
        }

        public void AddNodeLabel(Node node, Object label)
        {
            Node nodey = nodes.Find(nodex => nodex.Id == node.Id);

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
            int countEdges = CountEdges() / 4;
            int maxEdges = (countNodes * (countNodes - 1)) / 2;

            if (IsEmpty() || maxEdges < countEdges)
            {
                return false;
            }

            foreach (Node node in nodes)
            {
                foreach (Node otherNode in nodes)
                {
                    if (node != otherNode)
                    {
                        if (!ExistsEdge(node, otherNode))
                        {
                            return false;
                        }
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
            if (IsComplete())
            {
                return;
            }
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
            for (int i = 0; i < numNodes; i++)
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

        public static Graph GenerateConnectedGraph(int numVertices)
        {
            Graph graph = new Graph();

            object lockObj = new object();

            Parallel.For(0, numVertices, i =>
            {
                Node node = new Node(i);
                lock (lockObj) {
                    graph.AddNode(node);

                }
                Console.WriteLine("n");
            });

            Parallel.For(0, numVertices, i =>
            {
                Console.WriteLine("e");
                Node node = graph.GetNodeById(i);
                Node nextNode = graph.GetNodeById((i + 1) % numVertices);

                int nextIndex = (i + 1) % numVertices;

                if (nextIndex < 0)
                {
                    nextIndex += numVertices;
                }
                nextNode = graph.GetNodeById(nextIndex);

                lock (lockObj)
                {
                    graph.AddEdge(node, nextNode);
                }
            });

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
            if (IsComplete())
            {
                return;
            }
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

        public List<Edge> UniqueEdges()
        {
            List<Edge> uniqueEdges = new List<Edge>();

            foreach (Edge edge in edges)
            {
                bool exists = uniqueEdges.Any(e =>
                    (e.Origem == edge.Origem && e.Destino == edge.Destino) ||
                    (e.Origem == edge.Destino && e.Destino == edge.Origem));

                if (!exists)
                {
                    uniqueEdges.Add(edge);
                }
            }

            return uniqueEdges;
        }

        public List<Edge> GetEdgesOrigin(Node node)
        {
            return edges.FindAll(e => e.Origem == node);
        }

        public List<Node> FleuryTarjan()
        {
            /*if (!isConnected())
            {
                Console.WriteLine("Grafo desconexo.");
                return null;
            }
            if (MoreThanThreeOddNodes())
            {
                Console.WriteLine("Existem mais de três vértices de grau ímpar, tornando impossível existir um caminho Euleriano.");
                return null;
            }*/

            Console.WriteLine("aaqqi");
            Graph tempGraph = new Graph();
            Stopwatch sw = Stopwatch.StartNew();
            sw.Start();
            tempGraph.nodes.AddRange(nodes);
            tempGraph.edges.AddRange(uniqueEdges);
            sw.Stop();

            Console.WriteLine(sw.Elapsed);
            sw.Restart();
            Console.WriteLine("xxxx");
            //List<Node> oddNodes = GetNodesWithOddDegree();
            sw.Stop();
            Console.WriteLine(sw.Elapsed);


            //Node startNode = (oddNodes.Count > 0) ? oddNodes[0] : nodes[0];
            Node startNode = nodes[0];
            List<Node> circuit = new List<Node>();
            Stack<Node> stack = new Stack<Node>();
            stack.Push(startNode);

            while (stack.Count > 0)
            {
                Node currentNode = stack.Peek();
                Console.WriteLine(currentNode.Id);
                sw.Restart();
                List<Edge> incidentEdges = tempGraph.GetEdgesOrigin(currentNode);


                if (incidentEdges.Count > 0)
                {
                    Edge chosenEdge = null;

                    foreach (Edge edge in incidentEdges)
                    {
                        if (!IsBridgeTarjan(tempGraph, edge))
                        {
                            chosenEdge = edge;
                            break;
                        }
                    }
                    if (chosenEdge == null)
                    {
                        chosenEdge = incidentEdges[0];
                    }

                    Node nextNode = chosenEdge.Destino;

                    tempGraph.removeEdge(currentNode, nextNode);

                    stack.Push(nextNode);
                    sw.Stop();
                    Console.WriteLine(sw.Elapsed);
                }
                else
                {
                    stack.Pop();
                    circuit.Add(currentNode);
                }
            }

            circuit.Reverse();
            return circuit;
        }

        public List<Node> FleuryNaive()
        {
            /*if (!isConnected())
            {
                Console.WriteLine("Grafo desconexo.");
                return null;
            }
            if (MoreThanThreeOddNodes())
            {
                Console.WriteLine("Existem mais de três vértices de grau ímpar, tornando impossível existir um caminho Euleriano.");
                return null;
            }*/

            Console.WriteLine("aaqqi");
            Graph tempGraph = new Graph();
            Stopwatch sw = Stopwatch.StartNew();
            sw.Start();
            tempGraph.nodes.AddRange(nodes);
            tempGraph.edges.AddRange(uniqueEdges);
            sw.Stop();

            Console.WriteLine(sw.Elapsed);
            sw.Restart();
            Console.WriteLine("xxxx");
            //List<Node> oddNodes = GetNodesWithOddDegree();
            sw.Stop();
            Console.WriteLine(sw.Elapsed);


            //Node startNode = (oddNodes.Count > 0) ? oddNodes[0] : nodes[0];
            Node startNode = nodes[0];
            List<Node> circuit = new List<Node>();
            Stack<Node> stack = new Stack<Node>();
            stack.Push(startNode);

            while (stack.Count > 0)
            {
                Node currentNode = stack.Peek();
                Console.WriteLine(currentNode.Id);
                sw.Restart();
                List<Edge> incidentEdges = tempGraph.GetEdgesOrigin(currentNode);


                if (incidentEdges.Count > 0)
                {
                    Edge chosenEdge = null;

                    foreach (Edge edge in incidentEdges)
                    {
                        if (!IsBridgeNaive(tempGraph, edge))
                        {
                            chosenEdge = edge;
                            break;
                        }
                    }
                    if (chosenEdge == null)
                    {
                        chosenEdge = incidentEdges[0];
                    }

                    Node nextNode = chosenEdge.Destino;

                    tempGraph.removeEdge(currentNode, nextNode);

                    stack.Push(nextNode);
                    sw.Stop();
                    Console.WriteLine(sw.Elapsed);
                }
                else
                {
                    stack.Pop();
                    circuit.Add(currentNode);
                }
            }

            circuit.Reverse();
            return circuit;
        }

        public List<Edge> GetUniqueEdges(Node node)
        {
            List<Edge> unique = UniqueEdges();
            return edges.FindAll(edge => edge.Origem == node || edge.Destino == node);
        }
        private List<Node> GetNodesWithOddDegree()
        {
            List<Node> oddNodes = new List<Node>();
            foreach (Node node in nodes)
            {
                if (GetUniqueEdges(node).Count % 2 != 0)
                {
                    oddNodes.Add(node);
                }
            }
            return oddNodes;
        }

        private bool MoreThanThreeOddNodes()
        {
            int oddDegreeCount = 0;
            foreach (Node node in nodes)
            {
                if (GetEdges(node).Count % 2 != 0)
                {
                    oddDegreeCount++;
                }
            }
            return oddDegreeCount > 3;
        }

        private void AddEdgeNaive(Node origem, Node destino, int peso = 1)
        {
            Edge newEdge = new Edge(origem, destino, peso);
            edges.Add(newEdge);
            uniqueEdges.Add(newEdge);

            if (!isDirected)
            {
                Edge inverseEdge = new Edge(destino, origem, peso);
                edges.Add(inverseEdge);
            }
        }
        private bool IsBridgeNaive(Graph tempGraph, Edge edge)
        {
            int initialComponents = CountComponents(tempGraph); // Obtem o número inicial de componentes
            tempGraph.removeEdge(edge.Origem, edge.Destino);
            bool isConnected = tempGraph.isConnected();
            int currentComponents = CountComponents(tempGraph); // Obtem o número de componentes após a remoção

            tempGraph.AddEdgeNaive(edge.Origem, edge.Destino);

            return currentComponents > initialComponents && !isConnected;
        }

        private int CountComponents(Graph graph)
        {
            int k = 0;

            // Itera sobre todos os vértices do grafo de forma paralela
            Parallel.ForEach(graph.nodes, node =>
            {
                if (graph.GetEdges(node).Count == 0)
                {
                    Interlocked.Increment(ref k);
                }
            });

            return k;
        }

        private bool IsBridgeTarjan(Graph graph, Edge edge)
        {
            List<Edge> ed = graph.FindBridgesTarjan();
            return ed.Any(e => e.Equals(edge));
        }

        public static void MeasureExecutionTime(Action method, String methodName)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();

            method();

            stopwatch.Stop();

            Console.WriteLine("Tempo de execução do método " + methodName + " : "+  stopwatch.ElapsedMilliseconds);
        }

        public void printedges()
        {
            foreach (Edge edge in edges)
            {
                Console.WriteLine(edge.Origem.Id + "---" + edge.Destino.Id);
            }
        }
    }

}

