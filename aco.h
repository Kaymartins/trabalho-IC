#ifndef ACO_H
#define ACO_H

#include "Graph.h"
#include <queue>
#include <iostream>

typedef struct{
    vector<Node*> antSolution;
    double solutionCost;
    bool isBestSolution;
} Ant;

void initializePheromones(Graph &graph, double pheromone){
    Node* currentNode = graph.getFirstNode();

    while(currentNode != nullptr){
        currentNode->setPheromone(pheromone);
        currentNode = currentNode->getNextNode();
    }
}

void updatePheromones(const vector<Ant> &ants, double evaporationRate){
    for(Ant ant : ants){
        if(ant.isBestSolution){
            for(Node* node : ant.antSolution){
                node->setPheromone((1 - evaporationRate)*node->getPheromone() + (evaporationRate/ant.solutionCost));
            }
        }else{
            for(Node* node : ant.antSolution){
                node->setPheromone((1 - evaporationRate)*node->getPheromone());
            }
        }
    }
}

void initializeAnts(vector<Ant> &ants, int numberOfAnts, Node* randomNode){
    for(int i = 0; i < numberOfAnts; i++){
        Ant ant;
        ant.antSolution.push_back(randomNode);
        ant.solutionCost = randomNode->getWeight();
        ant.isBestSolution = false;
        ants.push_back(ant);
    }
}






#endif // ACO_H
