#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED

#include "Node.h"
#include <vector>
#include <queue>
#include <chrono>
#include <map>

/**
 * Struct para comparar valores com base no valor float do par para se obter uma min heap
*/
struct Compare
{
    bool operator()(const std::pair<float, int> &a, const std::pair<float, int> &b)
    {
        // Comparação com base no valor float do par
        return a.first > b.first; // '>' para obter uma min heap
    }
};

/*
 * Struct para gerar métricas dos algoritmos e auxiliar na criação do relatório
*/
struct Metric
{
    float time;
    int numberOfNodes;
    int totalWeight;
    int bestAlpha;
};

class Graph
{
private:
    int order;
    int numberOfEdges;
    int totalOfEdges;
    int uncoveredEdges;

    Node *firstNode;
    Node *lastNode;
    Node *removedNode;
    bool weightedEdges;
    bool weightedNodes;

    bool directed;
    map<int, Node *> nodeMap;
    map<int, vector<int>> openNeighborhoodMap;
    vector<pair<float, int>> *candidates;

public:
    Graph(int order, bool directed, bool weightedEdges, bool weightedNodes, int totalOfEdges);
    ~Graph();

    //métodos relacionados a informações do grafo
    
    int getOrder();
    int getNumberOfEdges();
    Node *getFirstNode();
    Node *getLastNode();
    int getUncoveredEdges();
    pair<int, int> getNodeDegree(int id);
    vector <pair<float, int>> *getCandidates();
    map<int, Node *> getNodeMap();

    void decrementUnmarkedEdges(int number);
    void setUncoveredEdges(int number);

        // métodos relacionados a caracteristicas do grafo

    bool isWeightedEdges();
    bool isWeightedNodes();
    bool isDirected();

    //métodos relacionados a manipulação do grafo

    void addNode(int id, float weight);
    void addEdge(int id, int targetId, float weight);
    Node *searchNode(int id);
    void removeNode(int id);
    void removeEdge(int id, int targetId);
    void removeAllEdges(int id);
    void resetMarks();

    void printGraph();

    bool isIsolated(); // Grafo sem arestas
    vector<int> getOpenNeighborhood(int id);
    vector<int> getClosedNeighborhood(int id);

    int getNumberOfUnmarkedEdges(Node *node);
    vector<int> getNeighbors(int id);

    //Métodos relacionados a lista de candidatos

    void createCandidates();
    void updateCandidates(int removedNodeId);
    void createNeighborhoodMap();
    map<int, vector<int>> getNeighborhoodMap();
};

#endif // GRAPH_H_INCLUDED