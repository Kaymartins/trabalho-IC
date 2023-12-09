#include "Graph.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <map>
#include <queue>
#include <cmath>
#include <random>

using namespace std;

Graph::Graph(int order, bool directed, bool weightedEdges, bool weightedNodes, int totalOfEdges)
{
    this->order = order;
    this->numberOfEdges = 0;
    this->firstNode = nullptr;
    this->lastNode = nullptr;
    this->weightedEdges = weightedEdges;
    this->weightedNodes = weightedNodes;
    this->directed = directed;
    this->totalOfEdges = totalOfEdges;
    this->uncoveredEdges = totalOfEdges;
}

Graph::~Graph()
{
    // Destruir os nós relacionados ao Graph
    Node *currentNode = this->firstNode;

    while (currentNode != nullptr)
    {
        Node *nextNode = currentNode->getNextNode();
        delete currentNode;
        currentNode = nextNode;
    }
}

int Graph::getOrder() { return this->order; }

int Graph::getNumberOfEdges() { return this->numberOfEdges; }

Node *Graph::getFirstNode() { return this->firstNode; }

Node *Graph::getLastNode() { return this->lastNode; }

bool Graph::isWeightedEdges() { return this->weightedEdges; }

bool Graph::isWeightedNodes() { return this->weightedNodes; }

bool Graph::isDirected() { return this->directed; }


/*
 * Método responsável por encontrar um nó no grafo com base no id do nó
    * @param id id do nó
    * @return nó encontrado
 */

Node *Graph::searchNode(int id)
{
    Node *currentNode = this->firstNode;

    while (currentNode != nullptr)
    {
        if (currentNode->getId() == id)
        {
            return currentNode;
        }

        currentNode = currentNode->getNextNode();
    }

    return nullptr;
}

/*
 * Método responsável por adicionar um nó no grafo
    * @param id id do nó
    * @param weight peso do nó
    * @return void
 */
void Graph::addNode(int id, float weight)
{
    if (nodeMap[id] != nullptr)
    {
        return;
    }

    Node *newNode = new Node(id);
    newNode->setWeight(weight);
    nodeMap[newNode->getId()] = newNode;

    if (this->firstNode == nullptr)
    {
        this->firstNode = newNode;
        this->lastNode = newNode;
    }
    else
    {
        this->lastNode->setNextNode(newNode);
        this->lastNode = newNode;
    }
    this->order++;
}

/*
 * Método responsável por adicionar uma aresta no grafo
    * @param id id do nó
    * @param targetId id do nó alvo
    * @param weight peso da aresta
    * @return void
 */
void Graph::addEdge(int id, int targetId, float weight)
{
    Node *node = nodeMap[id];
    Node *targetNode = nodeMap[targetId];

    if (node == nullptr || targetNode == nullptr)
    {
        cout << "Nó não encontrado" << endl;
        return;
    }

    if (node->searchEdge(targetId) != nullptr)
    {
        return;
    }
    node->addEdge(targetNode, this->directed, weight);
    node->incrementDegree(this->directed);
    node->incrementNumberOfEdges();

    if (!this->directed)
    {
        if (targetNode->searchEdge(id) == nullptr)
        {
            targetNode->addEdge(node, this->directed, weight);
            targetNode->incrementDegree(this->directed);
        }
    }

    this->numberOfEdges++;
}

/*
 * Método responsável remover um nó do grafo
    * @param id id do nó
    * @return void
 */
void Graph::removeNode(int id)
{
    Node *currentNode = this->firstNode;
    Node *previousNode = nullptr;

    while (currentNode != nullptr)
    {

        currentNode->removeEdge(id);
        cout << "Removendo aresta do nó " << currentNode->getId() << " para o nó " << id << endl;
        if (currentNode->getId() == id)
        {
            if (previousNode == nullptr)
            {
                this->firstNode = currentNode->getNextNode();
            }
            else
            {
                previousNode->setNextNode(currentNode->getNextNode());
            }
        }
        previousNode = currentNode;
        currentNode = currentNode->getNextNode();
    }

    delete currentNode;
    this->order--;
    return;
}

/*
 * Método responsável por remover uma aresta do grafo
    * @param id id do nó
    * @param targetId id do nó alvo
    * @return void
 */
void Graph::removeEdge(int id, int targetId)
{
    Node *node = this->nodeMap[id];

    if (node == nullptr)
    {
        return;
    }

    Node *targetNode = this->nodeMap[targetId];
    node->removeEdge(targetNode);
    targetNode->removeEdge(node);
    node->decrementNumberOfEdges();
    targetNode->decrementNumberOfEdges();
    this->numberOfEdges--;
}

/*
 * Método responsável por remover todos as arestas do grafo
    * @param id id do nó
    * @return void
 */
void Graph::removeAllEdges(int id)
{
    Node *node = this->nodeMap[id];

    if (node == nullptr)
    {
        return;
    }

    Edge *currentEdge = node->getFirstEdge();
    Edge *aux = nullptr;

    while (currentEdge != nullptr)
    {
        aux = currentEdge;
        currentEdge = currentEdge->getNextEdge();
        this->removeEdge(id, aux->getTargetId());
    }
}


/*
 * Método que retorna a vizinhaça aberta de um nó
    * @param id id do nó
    * @return vetor com os ids dos nós vizinhos
 */
vector<int> Graph::getOpenNeighborhood(int id)
{
    Node *search = nodeMap[id];

    if (search == nullptr)
    {
        return vector<int>();
    }

    Node *node = this->firstNode;
    vector<int> neighborhood;

    while (node != nullptr)
    {
        Edge *edge = node->searchEdge(id);

        if (edge != nullptr)
        {
            neighborhood.push_back(node->getId());
        }

        node = node->getNextNode();
    }

    return neighborhood;
}

/*
 * Método que retorna a vizinhaça fechada de um no
    * @param id id do nó
    * @return vetor com os ids dos nós vizinhos
 */
vector<int> Graph::getClosedNeighborhood(int id)
{
    vector<int> neighborhood = this->getOpenNeighborhood(id);
    neighborhood.push_back(id);

    return neighborhood;
}


/*
 * Método que remove todas marcações dos nós do grafo
    * @return void
 */
void Graph::resetMarks()
{
    int count = 0;
    Node *node = this->firstNode;

    while (node != nullptr)
    {
        node->setMarked(false);

        Edge *edge = node->getFirstEdge();
        while (edge != nullptr)
        {
            count++;
            edge->setMarked(false);
            edge = edge->getNextEdge();
        }
        node->setNumberOfUnmarkedEdges(count);
        count = 0;
        node = node->getNextNode();
    }

    this->uncoveredEdges = this->numberOfEdges;
}

/*
 * Função para marcar um vértice e suas arestas que saem dele
    * @param node nó a ser marcado
    * @return void
 */
void Graph::markNode(Node *node)
{
    node->setMarked(true);
    node->setNumberOfUnmarkedEdges(0);
    const vector<int> &neighbors = this->openNeighborhoodMap[node->getId()];

    int size = neighbors.size();
    int i = 0;
    for (int neighbor : neighbors)
    {
        Node *neighborNode = this->nodeMap[neighbor];
        if (!neighborNode->isMarked())
        {
            neighborNode->decrementUnmarkedEdges();
            this->uncoveredEdges--;
        }
    }
}

/*
 * Função para decrementar o número de arestas não marcadas
    * @return void
 */
void Graph::decrementUnmarkedEdges()
{
    this->uncoveredEdges--;
}

/*
 *Obtém os vizinhos de um nó, recebe o id do nó como parametro e retorna um vetor com os vizinhos do nó
    * @param id
    * @return vetor com os ids dos vizinhos do nó
 */
vector<int> Graph::getNeighbors(int id)
{
    Node *node = nodeMap[id];

    if (node == nullptr)
    {
        return vector<int>();
    }

    vector<int> neighbors;
    Edge *edge = node->getFirstEdge();

    while (edge != nullptr)
    {
        neighbors.push_back(edge->getTargetId());
        edge = edge->getNextEdge();
    }

    return neighbors;
}

/*
 * Função para verificar se a solução é viável ou não
 * Verifica se todas as arestas tem pelo menos um vértice marcado
    * @return true se a solução é viável e false caso contrário
 */
bool Graph::isIsolated()
{
    return this->uncoveredEdges == 0;
}

int Graph::getUncoveredEdges()
{
    return this->uncoveredEdges;
}

/*
 * Função para verificar se o grafo é trivial.
 * Verifica se o numero de vertices é igual a 1, se o primeiro nó nao é nulo, se o primeiro nó é igual ao ultimo nó
 * e se o grau do primeiro nó é igual a zero
    * @return true se todas verificações forem true e false caso alguma não seja.
 */

/*
 * Função para verificar o grau de um determinado nó do grafo
    * @param id
    * @return pair com o grau de entrada e o grau de saída do nó
 */
pair<int, int> Graph::getNodeDegree(int id)
{
    Node *node = this->searchNode(id);
    if (node == nullptr)
    {
        return make_pair(-1, -1);
    }

    return make_pair(node->getInDegree(), node->getOutDegree());
}

/*
 * Função para verificar se o grafo é nulo
 * percorre todos os nós e verifica se existe uma aresta adjacente a ele.
    * @return true se o grafo for nulo e false caso contrário  
 */

/*
 * Função que conta quantas arestas não marcadas um vértice possui, para fazer o cálculo do peso relativo
    * @param Node *node, nó que será verificado
    * @return int, que indica quantas arestas não marcadas o nó possui   
 */
int Graph::getNumberOfUnmarkedEdges(Node *node)
{
    Edge *edge = node->getFirstEdge();
    int numberOfUnmarkedEdges = 0;

    while (edge != nullptr)
    {
        Node *targetNode = nodeMap[edge->getTargetId()];
        if (!targetNode->isMarked() && !node->isMarked())
        {
            numberOfUnmarkedEdges++;
        }

        edge = edge->getNextEdge();
    }

    return numberOfUnmarkedEdges;
}

/*
 * Função que cria um mapa com a vizinhança aberta de cada nó
 * O mapa é alocado no atributo openNeighborhoodMap
    * @return void
 */
void Graph::createNeighborhoodMap()
{
    Node *aux = firstNode;
    while (aux != nullptr)
    {
        vector<int> neighborhood = getOpenNeighborhood(aux->getId());
        this->openNeighborhoodMap.insert(pair<int, vector<int>>(aux->getId(), neighborhood));
        aux = aux->getNextNode();
    }
}

map<int, vector<int>> Graph::getNeighborhoodMap()
{
    map<int, vector<int>> openNeighborhoodMap2;
    Node *aux = firstNode;
    while (aux != nullptr)
    {
        vector<int> neighborhood = getOpenNeighborhood(aux->getId());
        openNeighborhoodMap2.insert(pair<int, vector<int>>(aux->getId(), neighborhood));
        aux = aux->getNextNode();
    }
    return openNeighborhoodMap2;
}

    /*
     * Função que calcula o peso relativo de cada vértice
     * Colocamos os vértices em um vector ordenado, em que o vértice com menor peso relativo sempre esteja no topo
     * O vector criado é alocado no atributo candidates
     * @return void
     */

    void Graph::createCandidates()
{
    delete this->candidates;
    vector<pair<float, int>> *candidates = new vector<pair<float, int>>();
    this->candidates = candidates;
    Node *currentNode = this->firstNode;
    Node *relativeWeightNode = currentNode;

    while (currentNode != nullptr)
    {
        if (currentNode->isMarked())
        {
            currentNode = currentNode->getNextNode();
            continue;
        }

        candidates->push_back(make_pair(currentNode->getWeight() / currentNode->getNumberOfUnmarkedEdges(), currentNode->getId()));
        currentNode = currentNode->getNextNode();
    }
    sort(candidates->begin(), candidates->end(),
         [](pair<float, int> &a, pair<float, int> &b)
         {
             return a.first < b.first;
         });
}

vector<pair<float, int>> *Graph::getCandidates()
{
    createCandidates();
    return this->candidates;
}

map<int, Node *> Graph::getNodeMap()
{
    return this->nodeMap;
}

/*
 * Função que imprime o vetor de pesos relativos
    * @return void
 */

void Graph::printRelativeVector()
{
    ofstream myfile;
    myfile.open("relativeVector.txt", ios::trunc);
    for (int i = 0; i < candidates->size(); i++)
    {
        myfile << (*candidates)[i].first << " " << (*candidates)[i].second << endl;
    }
    myfile.close();
}

/*
 * Função que atualiza o vetor de pesos relativos
 * Quando um vértice é removido, o peso relativo dos seus vizinhos é atualizado
 *
 * O vetor é ordenado novamente
    * @param int removedNodeId, id do nó removido
    * @return void  
 */
void Graph::updateCandidates(int removedNodeId)
{
    if (candidates->size() == 0)
    {
        return;
    }
    // Percorre o vetor de vizinhos
    const vector<int> &neighbors = this->openNeighborhoodMap[removedNodeId];

    for (int i = 0; i < neighbors.size(); i++)
    {
        int neighborId = neighbors[i];

        // Encontra o índice do vizinho no vetor de pesos relativos
        auto it = find_if(candidates->begin(), candidates->end(),
                          [neighborId](const pair<float, int> &nodeWeight)
                          {
                              return nodeWeight.second == neighborId;
                          });

        if (it != candidates->end())
        {
            // Obtém o índice do vizinho
            int index = distance(candidates->begin(), it);

            // Atualize o peso relativo do vizinho
            candidates->at(index).first = nodeMap[neighborId]->getWeight() / nodeMap[neighborId]->getNumberOfUnmarkedEdges();
        }
    }

    // Ordena novamente o vetor de pesos relativos em ordem decrescente
    sort(candidates->begin(), candidates->end(),
         [](pair<float, int> &a, pair<float, int> &b)
         {
             return a.first < b.first;
         });
}

/*
 * Algoritmo Guloso Construtivo
 * Utiliza a heurística do peso relativo para construir uma solução viável
    * @return Metric, que indica o tempo de execução do algoritmo, o peso da solução e o número de nós presentes na solução	  
 */
Metric Graph::relativeHeuristic()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    start = chrono::high_resolution_clock::now();
    createNeighborhoodMap();

    // Conjunto solução inicial
    map<int, bool> solution;
    vector<int> solutionVector;

    for (int i = 0; i < this->getOrder(); i++)
    {
        solution.insert(make_pair(i, false));
    }

    createCandidates();

    bool viable = false;
    int firstHeuristcNode = candidates->front().second;
    float totalWeight = 0;
    int iterations = 0;
    while (!candidates->empty())
    {
        // Coloca o vértice na solução
        Node *node = nodeMap[firstHeuristcNode];
        solution[firstHeuristcNode] = true;
        solutionVector.push_back(firstHeuristcNode);
        totalWeight += node->getWeight();

        // Marca o vértice
        this->markNode(node);

        // Verifica se a solução é viável
        if (this->isIsolated())
        {
            viable = true;
            break;
        }

        candidates->erase(candidates->begin());
        candidates->shrink_to_fit();

        if (!candidates->empty())
        {
            updateCandidates(firstHeuristcNode);
            firstHeuristcNode = candidates->front().second;
        }
    }
    this->resetMarks();

    end = chrono::high_resolution_clock::now();
    float elapse_time = chrono::duration_cast<chrono::milliseconds>(end - start).count();

    Metric metric;
    metric.time = elapse_time;
    metric.totalWeight = totalWeight;
    metric.numberOfNodes = solutionVector.size();
    // Reseta os vértices para não marcados para gerar outras soluções
    this->resetMarks();
    return metric;
}

/*
 *  Função que imprime os resultados do algoritmo guloso construtivo no arquivo de saída
    * @param string output, nome do arquivo de saída
    * @param string instanceName, nome da instância
    * @return void
 */
void Graph::printSolution(string output, string instanceName)
{
    cout << "Iniciando algoritmo guloso construtivo" << endl;
    Metric metric = relativeHeuristic();
    cout << "Algoritmo guloso construtivo finalizado, resultados em: " << output << endl;
    ofstream file;
    file.open(output);
    file << "=============================" << endl;
    file << "Algoritimo Guloso Construtivo - " + instanceName << endl;
    file << "Tamanho da solução: " << metric.numberOfNodes << endl;
    file << "Peso total da solução: " << metric.totalWeight << endl;
    file << "Tempo de execução (ms): " << metric.time << endl;
    file << "=============================" << endl;

    file << endl;
    file.close();
}