#include <iostream>
#include <fstream>
#include "Graph.h"

using namespace std;

Graph *readGraph(string filename, bool isDirected, bool isWeightedEdges, bool isWeightedNodes)
{
    ifstream file(filename);

    if (!file.is_open())
    {
        cout << "Error opening file" << endl;
        return nullptr;
    }

    string order;

    getline(file, order);

    Graph *graph = new Graph(0, isDirected, isWeightedEdges, isWeightedNodes, 0);

    while (!file.eof())
    {
        int sourceId, targetId;
        float weight;

        file >> sourceId >> targetId >> weight;

        graph->addNode(sourceId, 0);
        graph->addNode(targetId, 0);

        graph->addEdge(sourceId, targetId, weight);
    }

    file.close();

    return graph;
}

Graph *readGreedy(string filename)
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

void menu(string input_file, string output_file, int numIter)
{
    Graph *graphGreedy = readGreedy(input_file);

    string instance = input_file.substr(input_file.find_last_of("/") + 1);
    instance = instance.substr(0, instance.find_last_of("."));

    float alphas[5] = {0.05, 0.1, 0.15, 0.3, 0.5};

    cout << "=============================" << endl;
    cout << "Trabalho de Inteligencia computacional" << endl;
    cout << "Instância: " << instance << endl;
    cout << "=============================" << endl;
    cout << "1 - Guloso Construtivo" << endl;
    cout << "0 - Sair" << endl;
    cout << "=============================" << endl;

    int option;
    cout << "Informe a opção desejada:" << endl;
    cin >> option;
    switch (option)
    {
    case 1:
        graphGreedy->printConstructiveGreedy(output_file, instance);
        break;
    case 0:
        cout << "Saindo..." << endl;
        break;
    default:
        cout << "Opção inválida" << endl;
        menu(input_file, output_file, numIter);
    }
    delete graphGreedy;
}

int main(int argc, char const *argv[])
{
    // Verificação se todos os parâmetros do programa foram entrados
    bool isDirected, isWeightedEdges, isWeightedNodes;
    int numIter;

    cout << "Uso do algoritmo: " << argv[0] << " <input_file> <output_file> <numIter>" << endl;
    if (argc == 4)
    {
        bool isDirected = argv[3];
        bool isWeightedEdges = argv[4];
        bool isWeightedNodes = argv[5];
        numIter = 0;
    }
    else
    {
        cout << "Número de parâmetros inválido" << endl;
        return 1;
    }

    string program_name = argv[0];
    string input_file = argv[1];
    string output_file = argv[2];

    menu(input_file, output_file, numIter);
    
    return 0;
}