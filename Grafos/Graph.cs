﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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
            
            if(edge1 != null && edge2 != null)
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
    }

}