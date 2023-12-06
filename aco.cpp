#include "aco.h"
#include "Graph.h"
#include <queue>
#include <iostream>

map<int, Node *> nodeMap;
map<int, vector<int>> openNeighborhoodMap;

void initializePheromones(Graph &graph, vector<pair<float, int>> *candidates)
{
    for (pair<float, int> candidate : *candidates)
    {
        Node *node = nodeMap[candidate.second];
        node->setPheromone(1 / candidate.first);

    }
}

void updateLocalPheromones(vector<Ant> &ants, double phi)
{
    for (Ant &ant : ants)
    {
        for (int nodeId : ant.antSolution)
        {
            Node *node = nodeMap[nodeId];
            double newPheromone = (1 - phi) * node->getPheromone() + phi * node->getInitialPheromone();
            node->setPheromone(newPheromone);
        }
    }
}

void updateGlobalPheromones(vector<Ant> &ants, double evaporationRate)
{
    for (Ant &ant : ants)
    {
        if (ant.isBestSolution)
        {
            for (int nodeId : ant.antSolution)
            {
                Node *node = nodeMap[nodeId];
                double newPheromone = (1 - evaporationRate) * node->getPheromone() + (evaporationRate / ant.solutionCost);
                node->setPheromone(newPheromone);
            }
        }
        else
        {
            for (int nodeId : ant.antSolution)
            {
                Node *node = nodeMap[nodeId];
                double newPheromone = (1 - evaporationRate) * node->getPheromone();
                node->setPheromone(newPheromone);
            }
        }
    }
}

void initializeAnts(vector<Ant> &ants, int numberOfAnts, Graph &graph)
{
    srand(time(NULL)); // Inicializa a semente do gerador de números aleatórios

    for (int i = 0; i < numberOfAnts; i++)
    {
        Ant ant;
        int randomNodeIndex = rand() % graph.getOrder(); // Escolhe um índice de nó aleatório

        ant.antSolution.push_back(randomNodeIndex); // Usa o índice do nó
        ant.solutionCost = nodeMap[randomNodeIndex]->getWeight();
        ant.isBestSolution = false;

        ants.push_back(ant);
    }
}

void aco(Graph &graph, int cycles, int steps, float evaporation, float alpha, float beta)
{
    vector<Ant> ants;
    int nAnts = graph.getOrder() * 0.25;
    openNeighborhoodMap = graph.getNeighborhoodMap();
    nodeMap = graph.getNodeMap();

    vector<pair<float, int>> *candidates = graph.getCandidates();
    initializePheromones(graph, candidates);
    initializeAnts(ants, nAnts, graph);

}