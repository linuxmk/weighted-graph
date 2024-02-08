#include "graph.h"
#include <queue>
#include <iterator>

Graph::Graph(const std::string& fileName)
{
    readGraph(fileName);
}

int Graph::readGraph(const std::string& fileName)
{
    std::ifstream inFile(fileName);
    if(!inFile.is_open())
    {
        std::cerr << "Can not open file "<< fileName << "\n";
        return -1;
    }

    size_t index = 0;

    std::vector<Edge>   edges;
    std::string         inputLine;

    while(std::getline(inFile, inputLine))
    {
        std::istringstream iss(inputLine);
        Edge_t edge;

        iss >> edge.startNode >> edge.distance >> edge.endNode;
        if(edge.distance < 0)
            return -1;

        if(mMapNodeToIndex.find(edge.startNode) == mMapNodeToIndex.end())
            mMapNodeToIndex[edge.startNode] = index++;

        if(mMapNodeToIndex.find(edge.endNode) == mMapNodeToIndex.end())
            mMapNodeToIndex[edge.endNode] = index++;

        edges.push_back(std::move(edge));
    }

    mSize = mMapNodeToIndex.size();
    mEdges.reserve(mSize);
    size_t i = 0;

    while(i < edges.size())
    {
        int index = mMapNodeToIndex[edges[i].startNode];
        mEdges[index].push_back(std::pair<float, std::string>(edges[i].distance, edges[i].endNode));
        ++i;
    }

    return 0;
}

float Graph::findMaxWeightPath(const std::string& startNode, const std::string& endNode)
{
    auto distance = dijkstra(startNode);
    if(distance.empty())
        return -1;

    if(mMapNodeToIndex.find(endNode) == mMapNodeToIndex.end())
        return -1;
    auto retVal = distance[mMapNodeToIndex[endNode]];
    if( retVal == __INT_MAX__)
        return -1;
    return -retVal;
}

std::vector<float> Graph::dijkstra(const std::string& startNode)
{
    std::priority_queue<std::pair<float, std::string>, std::vector<std::pair<float, std::string>>, std::greater<std::pair<float, std::string>>> pq;

    std::vector<float> distance(mSize, __INT_MAX__);

    pq.push({0,startNode});

    while(!pq.empty())
    {
        float dist = pq.top().first;
        std::string node = pq.top().second;
        if(mMapNodeToIndex.find(node) == mMapNodeToIndex.end())
            return {};

        size_t indexNode = mMapNodeToIndex[node];
//        std::cerr << "node = " << node << " index = " << indexNode << "\n";

        pq.pop();

        for(auto& xNode : mEdges[indexNode])
        {
            float dstNode = -xNode.first; // change sign to get the max
            std::string endNode = xNode.second;
            size_t idx = mMapNodeToIndex[endNode];
//            std::cerr << "node = " << endNode << " index = " << idx << "\n";
//            std::cerr << "dist + dstNode = " << dist + dstNode << "\n";

            if(dist + dstNode < distance[idx])
            {
                distance[idx] = (dist + dstNode);
                pq.push({dist+dstNode, endNode});
            }
        }
    }
    return distance;
}

