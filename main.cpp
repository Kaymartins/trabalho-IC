#include <iostream>
#include <fstream>
#include "Graph.h"

using namespace std;


Graph *readGraph(string filename)
{
    srand(time(NULL));
    ifstream file(filename);

    if (!file.is_open())
    {
        cout << "Error opening file" << endl;
        return nullptr;
    }

    string orderS;
    string numberOfEdgesS;
    file >> orderS >> numberOfEdgesS;
    int order = stoi(orderS);
    int numberOfEdges = stoi(numberOfEdgesS);

    Graph *graph = new Graph(0, false, false, false, numberOfEdges);

    cout << "Starting reading graph..." << endl;

    while (!file.eof())
    {
        char aux;
        int sourceId, targetId;

        file >> aux >> sourceId >> targetId;

        // Mudando o peso dos vértices apenas para comparação com o outro artigo
        // A Fast and Robust Heuristic Algorithm for the Minimum Weight Vertex Cover Problem

        graph->addNode(sourceId, (sourceId % 200) + 1);
        graph->addNode(targetId, (targetId % 200) + 1);

        graph->addEdge(sourceId, targetId, 0);
    }

    if (order == graph->getOrder() && numberOfEdges == graph->getNumberOfEdges())
    {
        cout << "Graph read successfully!" << endl;
    }
    else
    {
        cout << "Error reading graph." << endl;
    }

    file.close();

    return graph;
}


int main(int argc, char const *argv[])
{

    string program_name = argv[0];
    string input_file = argv[1];
    string output_file = argv[2];
    int numIter = stoi(argv[3]);
    string instance = input_file.substr(input_file.find_last_of("/") + 1);
    instance = instance.substr(0, instance.find_last_of("."));

    Graph *graphGreedy = readGraph(input_file);
    cout << "Inteligencia Computacional - MWVC" << endl;
    cout << "Instancia: " << instance << endl;
    cout << "Numero de vertices: " << graphGreedy->getOrder() << endl;
    cout << "Numero de arestas: " << graphGreedy->getNumberOfEdges() << endl;
    cout << "Numero de iteracoes: " << numIter << endl;

    
    return 0;
}