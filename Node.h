#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED

#include <iostream>
#include <vector>
#include "Edge.h"

using namespace std;

class Node
{
private:
    Edge *firstEdge;
    Edge *lastEdge;
    int id;
    unsigned int inDegree; // Caso o grafo não seja direcionado, inDegree = outDegree
    unsigned int outDegree;
    float weight;
    Node *nextNode;
    int numberOfEdges;
    int numberOfUnmarkedEdges;
    bool marked;

    double pheromone;
    double initialPheromone;

public:
    Node(int id);
    ~Node();

    Edge *getFirstEdge();
    Edge *getLastEdge();
    int getId();
    unsigned int getInDegree();
    unsigned int getOutDegree();
    float getWeight();
    Node *getNextNode();
    int getNumberOfEdges();
    double getPheromone();
    double getInitialPheromone();
    void setInitialPheromone(double initialPheromone);

    bool hasEdgeTo(int targetId);
    bool isMarked();

    int getNumberOfUnmarkedEdges();
    void setNumberOfUnmarkedEdges(int numberOfUnmarkedEdges);
    void decrementUnmarkedEdges();

    void setNextNode(Node *nextNode);
    void setWeight(float weight);
    void incrementDegree(bool directed);
    void incrementNumberOfEdges();
    void decrementNumberOfEdges();
    void setMarked(bool marked);
    void setPheromone(double pheromone);

    Edge *searchEdge(int targetId);

    void addEdge(Node *target, bool directed, float weight);
    void removeEdge(Node *target);
    void removeEdge(int id);
    void removeAllEdges();
};

#endif // NODE_H_INCLUDED