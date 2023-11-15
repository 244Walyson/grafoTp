﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.WebSockets;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using ClosedXML.Excel;
using System.IO;
using System.IO.Enumeration;
using DocumentFormat.OpenXml.EMMA;

namespace Grafos
{
    public class Node
    {
        public Object Id { get; set; }
        public Node(Object id)
        {
            Id = id;
        }
    }
    public class Edge
    {
        public Node Origem { get; set; }
        public Node Destino { get; set; }
        public int Peso { get; set; }
        public Object Rotulo { get; set; }

        public Edge(Node origem, Node destino, int peso = 1, object rotulo = null)
        {
            Origem = origem;
            Destino = destino;
            Peso = peso;
            Rotulo = rotulo;
        }

        public void addRotulo(Object rotulo)
        {
            Rotulo = rotulo;
        }
    }

    public class Graph
    {
        private List<Node> nodes;
        private List<Edge> edges;
        public int[,] adjacencyMatrix { get; set; }
        public bool isDirected { get; set; }
        private object lockObj = new object();

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

        public void addEdge(Node origem, Node destino, int peso = 1)
        {
            Edge newEdge = new Edge(origem, destino, peso);
            edges.Add(newEdge);
            if (!isDirected)
            {
                Edge inverseEdge = new Edge(destino, origem, peso);
                edges.Add(inverseEdge);
            }
        }

        public void getAdjacencyMatriz()
        {
            adjacencyMatrix = new int[nodes.Count, nodes.Count];

            for (int i = 1; i < nodes.Count; i++)
            {
                for (int j = 1; j < nodes.Count; j++)
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

        public void addEdgeLabel(Node node1, Node node2, Object label)
        {
            Edge edge = edges.Find(edge => (edge.Origem == node1 && edge.Destino == node2) || (edge.Origem == node2 && edge.Destino == node1));
            edge.addRotulo(label);
        }

        public void removeEdge(Node source, Node destiny)
        {
            Edge edgeToRemove = edges.Find(edge => edge.Origem == source && edge.Destino == destiny);
            Edge edgeNotDirected = edges.Find(edge => edge.Destino == source && edge.Origem == destiny);
            // Remover a aresta se encontrada
            if (edgeToRemove != null)
            {
                edges.Remove(edgeToRemove);
                if (!isDirected)
                {
                    edges.Remove(edgeNotDirected);
                }
            }
        }
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

        public bool AreEdgesAdjacent(Object a, Object b)
        {
            Edge edge1 = edges.Find(edge => edge.Rotulo == a);
            Edge edge2 = edges.Find(edge => edge.Rotulo == b);

            if (edge1 != null && edge2 != null)
            {
                return edge1.Origem == edge2.Origem || edge1.Destino == edge2.Destino ||
                   edge1.Origem == edge2.Destino || edge1.Destino == edge2.Origem;
            }
            return false;
        }

        public bool ExistsEdge(Node source, Node destiny)
        {
            return edges.Any(edge => (edge.Origem == source && edge.Destino == destiny) || (edge.Origem == destiny && edge.Destino == source));
        }

        public int CountEdges()
        {
            return edges.Count;
        }

        public int CountNodes()
        {
            return nodes.Count;
        }

        public bool AreCompletGraph()
        {
            int countNodes = CountNodes();
            int countEdges = CountEdges() / 2;
            int maxEdges = (countNodes * (countNodes - 1)) / 2;

            if (AreEmptyGraph() || maxEdges < countEdges)
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


        public bool AreEmptyGraph()
        {
            return edges.Count == 0;
        }

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

        public List<Edge> FindBridgeNaive()
        {
            
            List<Edge> bridges = new List<Edge>();
            List<Edge> edgesCopy = new List<Edge>();

            foreach (var edge in edges)
            {
                // Verifica se a aresta já existe no resultado e se é uma aresta inversa
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
                addEdge(edge.Origem, edge.Destino, edge.Peso);
            }
            return bridges;
        }

        public List<Edge> GetEdges(Node node)
        {
            return edges.FindAll(edge => edge.Origem == node || edge.Destino == node);
        }
        
        public List<Edge> FindBridgesTarjam()
        {
            List<Edge> bridges = new List<Edge>();

            TarjanBridges(bridges);

            return bridges;
        }
        private void TarjanBridges(List<Edge> bridges)
        {
            int[] disc = new int[nodes.Count];
            int[] low = new int[nodes.Count];
            bool[] visited = new bool[nodes.Count];
            int time = 0;

            Stack<int> stack = new Stack<int>();

            for (int i = 0; i < nodes.Count; i++)
            {
                if (!visited[i])
                {
                    stack.Push(i);
                    int parent = -1;
                    while (stack.Count > 0)
                    {
                        int u = stack.Peek();
                        if (!visited[u])
                        {
                            visited[u] = true;
                            disc[u] = low[u] = ++time;

                            foreach (var edge in GetEdges(nodes[u]))
                            {
                                int v = nodes.IndexOf(edge.Destino);
                                if (v != -1 && !visited[v])
                                {
                                    stack.Push(v);
                                    parent = u;
                                }
                                else if (v != -1 && v != parent)
                                {
                                    low[u] = Math.Min(low[u], disc[v]);
                                }
                            }
                        }
                        else
                        {
                            stack.Pop();
                            foreach (var edge in GetEdges(nodes[u]))
                            {
                                int v = nodes.IndexOf(edge.Destino);
                                if (v != -1 && v != parent)
                                {
                                    low[u] = Math.Min(low[u], low[v]);
                                    if (low[v] > disc[u])
                                    {
                                        lock (lockObj) {
                                            bridges.Add(edge);

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        public List<Edge> DFS()
        {
            List<Edge> visitedEdges = new List<Edge>();
            HashSet<Node> visitedNodes = new HashSet<Node>();

            foreach(var node in nodes)
            {
                if (!visitedNodes.Contains(node))
                {
                    DFSHelper(node, visitedNodes, visitedEdges);
                }
            }
            return visitedEdges;
        }

        private void DFSHelper(Node node, HashSet<Node> visitedNodes, List<Edge> visitedEdges)
        {
            visitedNodes.Add(node);

            foreach (var startNode in nodes)
            {
                if (!visitedNodes.Contains(startNode))
                {
                    Stack<Node> stack = new Stack<Node>();
                    stack.Push(startNode);

                    while (stack.Count > 0)
                    {
                        var currentNode = stack.Pop();
                        visitedNodes.Add(currentNode);

                        foreach (var edge in GetEdges(currentNode))
                        {
                            Node destination = (edge.Origem == currentNode) ? edge.Destino : edge.Origem;

                            if (!visitedNodes.Contains(destination))
                            {
                                visitedEdges.Add(edge);
                                stack.Push(destination);
                            }
                        }
                    }
                }
            }
            Console.WriteLine(node.Id);
        }

        public Node GetNodeById(Object id)
        {
            return nodes.Find(node => node.Id.Equals(id));
        }

        public static Graph GenerateRandomGraph(int nodes, int edges)
        {
            Random random = new Random();
            Graph graph = new Graph();

            for (int i = 0; i < nodes; i++)
            {
                Node node = new Node(i);
                graph.AddNode(node);
                Console.WriteLine(i);
            }
            for(int i = 0; i < edges; i++)
            {
                Console.WriteLine(i);
                Node destino = graph.GetNodeById(random.Next(nodes));
                Node origem = graph.GetNodeById(random.Next(nodes));
                int weight = random.Next(10000);
                graph.addEdge(origem, destino, weight);
            }
            return graph;
        }

        public void GenerateXlsx()
        {
            getAdjacencyMatriz();
            using (var workbook = new XLWorkbook())
            {
                var worksheet = workbook.Worksheets.Add("grafo1");

                for (int i = 0; i < nodes.Count; i++)
                {
                    // Definir os rótulos das linhas e colunas
                    worksheet.Cell(1, i + 2).Value = nodes[i].Id.ToString(); // Rótulos das colunas (na primeira linha)
                    worksheet.Cell(i + 2, 1).Value = nodes[i].Id.ToString(); // Rótulos das linhas (na primeira coluna)

                    for (int j = 0; j < nodes.Count; j++)
                    {
                        // Preencher os valores da matriz (começando da linha 2 e coluna 2)
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
        //print graph ---------------------------------------------------------
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

        public void PrintBridgesNaive()
        {
            List<Edge> bridges = FindBridgeNaive();
            foreach (var edge in bridges)
            {
                Console.WriteLine(edge.Origem.Id + " --> " + edge.Destino.Id);
            }
        }

        public void printBridgestarjam()
        {
            foreach(var edge in FindBridgesTarjam())
            {
                Console.WriteLine(edge.Origem.Id + " --> " + edge.Destino.Id);
            }
        }


        public List<Edge> FindBridgesTarjanMultithreaded()
        {
            List<Edge> bridges = new List<Edge>();

            TarjanBridgesMult(bridges);

            return bridges;
        }

        private void TarjanBridgesMult(List<Edge> bridges)
        {
            int[] disc = new int[nodes.Count];
            int[] low = new int[nodes.Count];
            bool[] visited = new bool[nodes.Count];
            int time = 0;

            Stack<int> stack = new Stack<int>();

            object lockObj = new object(); // Objeto de bloqueio para sincronização

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


        public static Graph GenerateRandomGraphMultithreaded(int nodes, int edges, int numThreads)
        {
            Graph graph = new Graph();
            List<Thread> threads = new List<Thread>();
            object lockObj = new object();

            for (int i = 0; i < numThreads; i++)
            {
                Thread thread = new Thread(() =>
                {
                    Random random = new Random();
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
                        Node destino = graph.GetNodeById(random.Next(nodes));
                        Node origem = graph.GetNodeById(random.Next(nodes));
                        int weight = random.Next(10000);
                        lock (lockObj)
                        {
                            graph.addEdge(origem, destino, weight);
                        }
                    }
                });

                threads.Add(thread);
            }

            // Inicia as threads
            foreach (var thread in threads)
            {
                thread.Start();
            }

            // Aguarda todas as threads terminarem
            foreach (var thread in threads)
            {
                thread.Join();
            }

            return graph;
        }

        public void printnodeid()
        {
            foreach (var node in nodes)
            {
                Console.WriteLine(node.Id.ToString()+"xx");
            }
        }


    }

}
