#include "aco.h"
#include "Graph.h"
#include <queue>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <map>
#include <cmath>
#include <random>

map<int, Node *> nodeMap;
map<int, vector<int>> openNeighborhoodMap;

void initializePheromones(Graph &graph, vector<pair<float, int>> *firstCandidates)
{
    for (pair<float, int> candidate : *firstCandidates)
    {
        Node *node = nodeMap[candidate.second];
        node->setPheromone(1 / candidate.first);

    }
}

void updateLocalPheromones(int nodeId, double phi)
{
    Node *node = nodeMap[nodeId];
    double newPheromone = (1 - phi) * node->getPheromone() + phi * node->getInitialPheromone();
    node->setPheromone(newPheromone);
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

void initializeAnts(vector<Ant> &ants, int numberOfAnts, Graph &graph, vector<pair<float, int>> &candidates)
{
    srand(time(NULL)); // Inicializa a semente do gerador de números aleatórios
    ants.clear();
    for (int i = 0; i < numberOfAnts; i++)
    {
        Ant ant;
        int randomNodeIndex = rand() % graph.getOrder(); // Escolhe um índice de nó aleatório

        if (nodeMap[randomNodeIndex]->getWeight() == 0)
        {
            ant.antSolution.push_back(randomNodeIndex); // Usa o índice do nó
            ant.solutionCost = nodeMap[randomNodeIndex]->getWeight();
            ant.isBestSolution = false;
            ants.push_back(ant);
            //achar a posicao do No encontrado na lista candidates
            for (int j = 0; j < candidates.size(); j++)
            {
                if (candidates[j].second == randomNodeIndex)
                {
                    ant.positionFirstNode = j;
                    break;
                }
            }
        }
        else
            i--;
    }
}

vector<pair<float, int>> createCandidatesPheromones(int tamGraph)
{
    vector<pair<float, int>> candidates;
    Node *node;

    for(int i = 1; i <=tamGraph; i++)
    {
        node = nodeMap[i];
        pair<float, int> candidate;
        candidate.first = node->getPheromone();
        candidate.second = node->getId();
        candidates.push_back(candidate);
    }
        sort(candidates.begin(), candidates.end(),
         [](pair<float, int> &a, pair<float, int> &b)
         {
             return a.first > b.first;
         });
    return candidates;
}

void updateCandidatesPheromones(vector<pair<float, int>> &candidates,  Ant &ant, double q0, double beta)
{
        for (pair<float, int> &candidate : candidates)
    {
        double q = static_cast<double>(rand()) / RAND_MAX; // Valor aleatório entre 0 e 1

        if (q < q0)
        {
            // Escolhe o vértice com a maior trilha de feromônio (exploração)
            auto maxPheromone = max_element(candidates.begin(), candidates.end(),
                                            [](const pair<float, int> &a, const pair<float, int> &b) {
                                                return a.first < b.first;
                                            });

            candidate = *maxPheromone;
        }
        else
        {
            // Calcula a probabilidade de escolher cada vértice com base no feromônio e na heurística local
            double sum = 0.0;
            int currentNode = ant.antSolution.back();
            double pheromone = nodeMap[currentNode]->getPheromone();
            double heuristic = pow((nodeMap[currentNode]->getNumberOfUnmarkedEdges() / nodeMap[currentNode]->getWeight()), beta);
            sum += pheromone * heuristic;
            

            for (pair<float, int> &candidate : candidates)
            {
                int candidateNode = candidate.second;
                double pheromone = nodeMap[currentNode]->getPheromone();
                double heuristic = pow((nodeMap[currentNode]->getNumberOfUnmarkedEdges() / nodeMap[currentNode]->getWeight()), beta);
                double probability = (pheromone * heuristic) / sum;

                candidate.first = probability;
            }

            // Escolhe o vértice com base na probabilidade calculada (exploração e/ou intensificação)
            discrete_distribution<int> distribution(candidates.begin(), candidates.end());
            int chosenIndex = distribution(default_random_engine());
            candidate = candidates[chosenIndex];
        }
    }

    // Ordena novamente a lista de candidatos com base nas probabilidades atualizadas
    sort(candidates.begin(), candidates.end(),
         [](pair<float, int> &a, pair<float, int> &b) {
             return a.first > b.first;
         });
}

    void aco(Graph &graph, int cycles, int steps, float evaporation, float alpha, float beta)
{
    vector<Ant> ants;
    int nAnts = graph.getOrder() * 0.25;
    openNeighborhoodMap = graph.getNeighborhoodMap();
    nodeMap = graph.getNodeMap();

    vector<pair<float, int>> *firstCandidates = graph.getCandidates();
    initializePheromones(graph, firstCandidates);

    vector<pair<float, int>> candidates = createCandidatesPheromones(graph.getOrder());

    for (int i = 0; i < cycles; i++)
    {
        //distruibui as formigas
        initializeAnts(ants, nAnts, graph, candidates);
        for (int i = 0; i < nAnts; i++)
        {
            Ant &ant = ants[i];

            int bestNode = -1;
            double bestNodeCost = 0;

            int node = ant.antSolution.back();
            graph.markNode(nodeMap[node]);
            candidates.erase(candidates.begin() + ant.positionFirstNode);
            candidates.shrink_to_fit();
            while(!graph.isIsolated())
            {
                node = candidates[0].second;
                ant.solutionCost += nodeMap[node]->getWeight();
                graph.markNode(nodeMap[node]);
                candidates.erase(candidates.begin());
                candidates.shrink_to_fit();
                ant.antSolution.push_back(node);
                
            }
        } 
        graph.resetMarks();
        updateGlobalPheromones(ants, evaporation);  
    }
    
}

