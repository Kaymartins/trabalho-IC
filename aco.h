#ifndef ACO_H
#define ACO_H

#include "Graph.h"
#include <queue>
#include <iostream>

typedef struct{
    vector<int> antSolution;
    int solutionCost;
    bool isBestSolution;
    int positionFirstNode;
} Ant;

/* void initializePheromones(Graph &graph, vector<pair<float, int>> *candidates);
void updateGlobalPheromones(const vector<Ant> &ants, double evaporationRate);
void initializeAnts(vector<Ant> &ants, int numberOfAnts, Graph &graph);
*/
void aco(Graph &graph, int cycles, int steps, float evaporation, float alpha, float beta); 
#endif // ACO_H
