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
    float      distance;
    std::string endNode;
}Edge_t;

class Graph
{
public:
    Graph(const std::string& fileName);
    int readGraph(const std::string &fileName);
    float findMaxWeight(const std::string &startNode, const std::string &endNode);
    std::vector<std::pair<std::string, double> > printShortestPath(const std::string &startNode, const std::string &endNode);

private:
    std::vector<float> dijkstra(const std::string& startNode);
    std::vector<std::pair<std::string, double> > findMaxWeightPath(const std::string &startNode, const std::string& endNode);

    std::vector<Edge>                                               edges;
    std::vector<std::vector<std::pair<float, std::string>>>         mEdges;
    size_t                                                          mSize;
    std::unordered_map<std::string, size_t>                         mMapNodeToIndex;
};

#endif // GRAPH_H
