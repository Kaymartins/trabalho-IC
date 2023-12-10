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

void createNumberOfNeighborhoodNotMarkedMap(map<int, int> &markedMap)
{
    for (auto &node : nodeMap)
    {
        markedMap[node.first] = node.second->getNumberOfUnmarkedEdges();
    }
}

void markNode(Node *node, map<int, int> &neighborhoodNotMarkedMap)
{
    vector<int> neighbors = openNeighborhoodMap[node->getId()];
    int i;
    node->setMarked(true);
    neighborhoodNotMarkedMap[node->getId()] = 0;
    for (int neighbor : neighbors)
    {
        Node *neighborNode = nodeMap[neighbor];
        if (!neighborNode->isMarked())
        {
            neighborhoodNotMarkedMap[neighbor]--;
        }
    }
}

bool isGraphIsolated(map<int, int> &neighborhoodNotMarkedMap)
{
    bool isIsolated = true;
    for (auto &node : neighborhoodNotMarkedMap)
    {
        if (node.second > 0)
        {
            isIsolated = false;
            break;
        }
    }
    return isIsolated;
}

void resetMarks(map<int, int> &neighborhoodNotMarkedMap)
{
    for (auto &node : nodeMap)
    {
        node.second->setMarked(false);
        node.second->setNumberOfUnmarkedEdges(node.second->getNumberOfEdges());
    }

    for (auto &node : neighborhoodNotMarkedMap)
    {
        node.second = nodeMap[node.first]->getNumberOfUnmarkedEdges();
    }

}

void initializeAnts(vector<Ant> &ants, int numberOfAnts, Graph &graph, vector<pair<pair<float, float>, int>> &candidates)
{
    srand(time(NULL)); // Inicializa a semente do gerador de números aleatórios
    ants.clear();
    for (int i = 0; i < numberOfAnts; i++)
    {
        Ant ant;
        int randomNodeIndex = rand() % candidates.size(); // Escolhe um índice de nó aleatório
        if (randomNodeIndex == 0)
            randomNodeIndex = 1;

        ant.antSolution.push_back(randomNodeIndex); // Usa o índice do nó
        ant.solutionCost = nodeMap[randomNodeIndex]->getWeight();
        ant.isBestSolution = false;

        // achar a posicao do No encontrado na lista candidates
        for (int j = 0; j < candidates.size(); j++)
        {
            if (candidates[j].second == randomNodeIndex)
            {
                ant.positionFirstNode = j;
                break;
            }
        }
        ants.push_back(ant);
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

vector<pair<float,int>> createCandidatesPheromones(vector<pair<float, int>> *firstCandidates)
{
    vector<pair<float,int>> candidates;
    Node *node;

    int i = 1;
    //percorre a lista passada como parametro
    for (pair<float, int> candidate : *firstCandidates)
    {
        node = nodeMap[candidate.second];
        int j;
        candidate.first = node->getPheromone();
        candidate.second = node->getId();
        candidates.push_back(candidate);
        i++;
    }

    sort(candidates.begin(), candidates.end(),
         [](pair<float, int> &a, pair<float, int> &b)
         {
             return a.first > b.first;
         });

    return candidates;
}

// vector<pair<float, int>> createCandidatesPheromones(int tamGraph)
// {
//     vector<pair<float, int>> candidates;
//     Node *node;

//     for (int i = 1; i <= tamGraph; i++)
//     {
//         node = nodeMap[i];
//         int j;
//         pair<float, int> candidate;
//         cout << "Node " << i << " " << node->getPheromone() << endl;
//         candidate.first = node->getPheromone();
//         candidate.second = node->getId();
//         candidates.push_back(candidate);
//     }
//     sort(candidates.begin(), candidates.end(),
//          [](pair<float, int> &a, pair<float, int> &b)
//          {
//              return a.first > b.first;
//          });

//     return candidates;

// }

void updateCandidatesProbabilities(vector<pair<pair<float,float>, int>> &candidates, double q0, double beta, int uncoveredEdges)
{
    double sum = 0.0;
    for (pair<pair<float, float>, int> &candidate : candidates)
    {
        int candidateNode = candidate.second;
        double pheromone = nodeMap[candidateNode]->getPheromone();
        double heuristic = pow((uncoveredEdges / nodeMap[candidateNode]->getWeight()), beta);
        sum += pheromone * heuristic;
    }

    for (pair<pair<float, float>, int> &candidate : candidates)
    {
        double q = static_cast<double>(rand()) / RAND_MAX; // Valor aleatório entre 0 e 1
        int candidateNode = candidate.second;
        double pheromone = nodeMap[candidateNode]->getPheromone();
        double heuristic = pow((uncoveredEdges / nodeMap[candidateNode]->getWeight()), beta);

        if (q < q0)
        {
            // Escolhe o vértice com a maior trilha de feromônio (exploração)
            auto maxPheromone = std::max_element(candidates.begin(), candidates.end(),
                                                 [](const std::pair<pair<float, float>, int> &a, const std::pair<pair<float, float>, int> &b)
                                                 {
                                                     return a.first.first < b.first.first;
                                                 });

            if (candidateNode == maxPheromone->second)
            {
                candidate.first.first = 1;
            }
            else
            {
                candidate.first.first = 0;
            }
        }
        else
        {
            // Calcula a probabilidade de escolher cada vértice com base no feromônio e na heurística local
            double probability = (pheromone * heuristic) / sum;
            candidate.first.first = probability;
        }
    }

    // Ordena novamente a lista de candidatos com base nas probabilidades atualizadas
    std::sort(candidates.begin(), candidates.end(),
              [](std::pair<pair<float, float>, int> &a, std::pair<pair<float, float>, int> &b)
              {
                  return a.first.first > b.first.first;
              });
}

//feromonio, peso relativo, id
vector<pair<pair<float, float>, int>> createCandidates()
{
    vector<pair<pair<float, float>, int>> candidates;
    Node *node;

    for (auto &node : nodeMap)
    {
        int j;
        pair<pair<float, float>, int> candidate;

        candidate.first.second = node.second->getWeight() / node.second->getNumberOfEdges();
        candidate.second = node.second->getId();
        node.second->setInitialPheromone(1 / candidate.first.second);
        candidate.first.first = 1 / candidate.first.second;
        candidates.push_back(candidate);
    }

    sort(candidates.begin(), candidates.end(),
         [](pair<pair<float, float>, int> &a, pair<pair<float, float>, int> &b)
         {
             return a.first.first > b.first.first;
         });

    return candidates;
}

void aco(Graph &graph, int cycles, int steps, float evaporation, float alpha, float beta)
{
    vector<Ant> ants;
    double bestSolutionCost = std::numeric_limits<double>::max();
    int nAnts = graph.getOrder() * 0.25;
    openNeighborhoodMap = graph.getNeighborhoodMap();
    nodeMap = graph.getNodeMap();
    map<int, int> * neighborhoodNotMarkedMap = new map<int, int>();
    createNumberOfNeighborhoodNotMarkedMap(*neighborhoodNotMarkedMap);
    Ant bestAnt;
    int uncoveredEdges = graph.getOrder();
    for (int i = 0; i < cycles; i++)
    {
        cout << "Ciclo " << i << endl;

        vector<pair<pair<float, float>, int>> candidates = createCandidates();        

        initializeAnts(ants, nAnts, graph, candidates); // inicializa todas as formigas em um nó aleatorio
        
        for (int i = 0; i < nAnts; i++)
        {
            //vector<pair<float, int>> auxCandidates = createCandidatesPheromones(&candidates);

            Ant &ant = ants[i];
            vector<int> bestSolution;
            // Ant *bestAnt = nullptr; // ponteiro para a melhor formiga
            int node = ant.antSolution.back();
            ant.solutionCost = nodeMap[node]->getWeight();
            markNode(nodeMap[node], *neighborhoodNotMarkedMap);
            uncoveredEdges -= nodeMap[node]->getNumberOfEdges();
            // remove o primeiro nó da lista de candidates com base na posição dele na lista.
            candidates.erase(candidates.begin() + ant.positionFirstNode);
            candidates.shrink_to_fit();

            bool validSolution = false;
            
            while (!validSolution)
            {
                
                // seleciona um candidato com base na roleta
                double sum_of_fitness = std::accumulate(candidates.begin(), candidates.end(), 0.0,
                                                        [](double sum, const std::pair<pair<float, float>, int> &candidates)
                                                        {
                                                            return sum + candidates.first.first;
                                                        });

                double roulette = static_cast<double>(rand()) / RAND_MAX;
                double partialSum = 0.0;
                int selected_candidate_position = -1;
                // obtém posição do candidato selecionado
                for (int i = 0; i < candidates.size(); i++)
                {
                    partialSum += candidates[i].first.first / sum_of_fitness;
                    if (partialSum >= roulette)
                    {
                        selected_candidate_position = i;
                        break;
                    }
                }
                if(selected_candidate_position == -1){
                    selected_candidate_position = 0;
                }

                if (candidates.size() == 0)
                {
                    cout << "Candidatos vazios" << endl;
                    break;
                }

                node = candidates[selected_candidate_position].second;
                ant.solutionCost += nodeMap[node]->getWeight();
                markNode(nodeMap[node], *neighborhoodNotMarkedMap);
                candidates.erase(candidates.begin() + selected_candidate_position);
                candidates.shrink_to_fit();
                ant.antSolution.push_back(node);

                //updateLocalPheromones(node, 0.5);
                updateCandidatesProbabilities(candidates, 0.5, beta, uncoveredEdges);
                validSolution = isGraphIsolated(*neighborhoodNotMarkedMap);
            }
            if (ant.solutionCost < bestSolutionCost)
            {
                bestSolutionCost = ant.solutionCost;
                bestSolution = ant.antSolution;
                bestAnt = ant;
                bestAnt.isBestSolution = true;
            }
            else
            {
                ant.isBestSolution = false;
            }

            resetMarks(*neighborhoodNotMarkedMap);
            candidates.clear();
            candidates.shrink_to_fit();
            candidates = createCandidates();
            uncoveredEdges = graph.getOrder();
        }

        updateGlobalPheromones(ants, evaporation);
    }

    cout << "Melhor solução: " << endl;
    for (int i = 0; i < bestAnt.antSolution.size(); i++)
    {
        cout << bestAnt.antSolution[i] << " ";
    }
    cout << endl;
    cout << "Custo da melhor solução: " << bestAnt.solutionCost << endl;
}
