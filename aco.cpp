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
        if (randomNodeIndex == 0)
            randomNodeIndex = 1;

        ant.antSolution.push_back(randomNodeIndex); // Usa o índice do nó
        ant.solutionCost = nodeMap[randomNodeIndex]->getWeight();
        ant.isBestSolution = false;
        ants.push_back(ant);
        // achar a posicao do No encontrado na lista candidates
        for (int j = 0; j < candidates.size(); j++)
        {
            if (candidates[j].second == randomNodeIndex)
            {
                ant.positionFirstNode = j;
                break;
            }
        }
    }
}

vector<pair<float, int>> createCandidatesPheromones(int tamGraph)
{
    vector<pair<float, int>> candidates;
    Node *node;

    for (int i = 1; i <= tamGraph; i++)
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

void updateCandidatesProbabilities(vector<pair<float, int>> &candidates, double q0, double beta, int uncoveredEdges)
{
    double sum = 0.0;
    for (pair<float, int> &candidate : candidates)
    {
        int candidateNode = candidate.second;
        double pheromone = nodeMap[candidateNode]->getPheromone();
        double heuristic = pow((uncoveredEdges / nodeMap[candidateNode]->getWeight()), beta);
        sum += pheromone * heuristic;
    }

    for (pair<float, int> &candidate : candidates)
    {
        double q = static_cast<double>(rand()) / RAND_MAX; // Valor aleatório entre 0 e 1
        int candidateNode = candidate.second;
        double pheromone = nodeMap[candidateNode]->getPheromone();
        double heuristic = pow((uncoveredEdges / nodeMap[candidateNode]->getWeight()), beta);

        if (q < q0)
        {
            // Escolhe o vértice com a maior trilha de feromônio (exploração)
            auto maxPheromone = std::max_element(candidates.begin(), candidates.end(),
                                                 [](const std::pair<float, int> &a, const std::pair<float, int> &b)
                                                 {
                                                     return a.first < b.first;
                                                 });

            if (candidateNode == maxPheromone->second)
            {
                candidate.first = 1;
            }
            else
            {
                candidate.first = 0;
            }
        }
        else
        {
            // Calcula a probabilidade de escolher cada vértice com base no feromônio e na heurística local
            double probability = (pheromone * heuristic) / sum;
            candidate.first = probability;
        }
    }

    // Ordena novamente a lista de candidatos com base nas probabilidades atualizadas
    std::sort(candidates.begin(), candidates.end(),
              [](std::pair<float, int> &a, std::pair<float, int> &b)
              {
                  return a.first > b.first;
              });
}

void markNode(Node *node, Graph &graph)
{
    vector<int> neighbors = openNeighborhoodMap[node->getId()];
    node->setMarked(true);
    node->setNumberOfUnmarkedEdges(0);
    for (int neighbor : neighbors)
    {
        Node *neighborNode = nodeMap[neighbor];
        if (!neighborNode->isMarked())
        {
            neighborNode->decrementUnmarkedEdges();
            graph.decrementUnmarkedEdges();
        }
    }
}



bool isGraphIsolated(Graph &graph)
{
    return graph.getUncoveredEdges() == 0;
}

void calculateNumberOfEdges(Graph &graph)
{
    int numberOfEdges = 0, numberOfUnmarkedEdges = 0;
    vector<int> neighbors;
    for (auto &node : nodeMap)
    {
        neighbors = openNeighborhoodMap[node.first];
        numberOfEdges += neighbors.size();
        numberOfUnmarkedEdges += node.second->getNumberOfUnmarkedEdges();
    }

    cout << "Number of edges: " << numberOfEdges/2 << endl;
    cout << "Number of edges in graph: " << graph.getNumberOfEdges() << endl;
    cout << "Number of unmarked edges: " << numberOfUnmarkedEdges/2 << endl;
}

void aco(Graph &graph, int cycles, int steps, float evaporation, float alpha, float beta)
{
    vector<Ant> ants;
    int nAnts = graph.getOrder() * 0.25;
    openNeighborhoodMap = graph.getNeighborhoodMap();
    nodeMap = graph.getNodeMap();

    for (int i = 0; i < cycles; i++)
    {
        vector<pair<float, int>> *firstCandidates = graph.getCandidates();                  // pega todos os candidatos do grafo
        initializePheromones(graph, firstCandidates);                                       // deposita o feromonio inicial nos candidatos com base no peso relativo
        vector<pair<float, int>> candidates = createCandidatesPheromones(graph.getOrder()); // cria e ordena a lista de candidatos com base no feromonio inicial
        for (int i = 0; i < candidates.size(); i++)
            initializeAnts(ants, nAnts, graph, candidates); // inicializa todas as formigas em um nó aleatorio
        
        for (int i = 0; i < 1; i++)
        {
            Ant &ant = ants[i];
            float numberOfEdgesCovered = 0.00;
            vector<int> bestSolution;
            double bestSolutionCost = std::numeric_limits<double>::max();
            Ant *bestAnt = nullptr; // ponteiro para a melhor formiga
            int node = ant.antSolution.back();
            ant.solutionCost = nodeMap[node]->getWeight();
            numberOfEdgesCovered += (nodeMap[node]->getNumberOfUnmarkedEdges())/2;
            markNode(nodeMap[node], graph);
            // remove o primeiro nó da lista de candidates com base na posição dele na lista.
            candidates.erase(candidates.begin() + ant.positionFirstNode);
            candidates.shrink_to_fit();
            while (static_cast<int>(numberOfEdgesCovered) != graph.getNumberOfEdges())
            {
                // seleciona um candidato com base na roleta
                double sum_of_fitness = std::accumulate(candidates.begin(), candidates.end(), 0.0,
                                                        [](double sum, const std::pair<float, int> &candidate)
                                                        {
                                                            return sum + candidate.first;
                                                        });

                double roulette = static_cast<double>(rand()) / RAND_MAX;
                double partialSum = 0.0;
                int selected_candidate_position = -1;
                // obtém posição do candidato selecionado
                for (int i = 0; i < candidates.size(); i++)
                {
                    partialSum += candidates[i].first / sum_of_fitness;
                    if (partialSum >= roulette)
                    {
                        selected_candidate_position = i;
                        break;
                    }
                }

                node = candidates[selected_candidate_position].second;
                ant.solutionCost += nodeMap[node]->getWeight();
                numberOfEdgesCovered += (nodeMap[node]->getNumberOfUnmarkedEdges())/2;
                markNode(nodeMap[node], graph);
                candidates.erase(candidates.begin() + selected_candidate_position);
                candidates.shrink_to_fit();
                ant.antSolution.push_back(node);
                cout << "Number of edges covered: " << numberOfEdgesCovered << endl;
                // atualiza a lista de candidatos com base no feromonio e na heuristica local
                //updateCandidatesProbabilities(candidates, 0.5, beta, graph.getUncoveredEdges());                
            }
            if (ant.solutionCost < bestSolutionCost)
            {
                if (bestAnt != nullptr)
                {
                    bestAnt->isBestSolution = false;
                }

                bestSolutionCost = ant.solutionCost;
                bestSolution = ant.antSolution;
                bestAnt = &ant;
                bestAnt->isBestSolution = true;
            }
            graph.resetMarks();
        }
        updateGlobalPheromones(ants, evaporation);
    }

/*     for (int i = 0; i < ants.size(); i++)
    {
        cout << "Ant " << i << ": ";
        for (int j = 0; j < ants[i].antSolution.size(); j++)
        {
            cout << ants[i].antSolution[j] << " ";
        }
        cout << "Cost: " << ants[i].solutionCost << " IsBestSolution: " << ants[i].isBestSolution << endl;
        cout << "------------------------------------";
        cout << endl;
    }
     */
}
