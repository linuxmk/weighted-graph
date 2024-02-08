#ifndef GRAPH_H
#define GRAPH_H
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <functional>
#include <cstring>
#include <unordered_map>

typedef struct  Edge
{
    std::string startNode;
    std::string endNode;
    float      distance;
}Edge_t;

class Graph
{
public:
    Graph(const std::string& fileName);
    float findMaxWeightPath(const std::string &startNode, const std::string &endNode);
    int readGraph(const std::string &fileName);
    std::vector<int> shortestPath(const std::string &startNode, int m);
private:
    std::vector<float> dijkstra(const std::string& startNode);

    std::vector<std::vector<std::pair<float, std::string>>>         mEdges;
    size_t                                                          mSize;
    std::unordered_map<std::string, size_t>                         mMapNodeToIndex;
};

#endif // GRAPH_H
