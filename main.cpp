#include "graph.h"

int main(int argc, char *argv[])
{

    if(argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " startNode endNode\n";
        return 1;
    }

    Graph g("graph.txt");

    auto retVal = g.findMaxWeight(argv[1], argv[2]);

    if(retVal != -1)
    {
        std::cerr << "PAth exits and its weight is " << retVal << "\n";
        auto path = g.printShortestPath(argv[1], argv[2]);

        if (path.empty())
        {
            std::cerr << "No path exists between " << argv[1] << " and " << argv[2] << "\n";
        }
        else
        {
            std::cerr << "Path with maximum weight between " << argv[1] << " and " << argv[2] << ":" << "\n";
            for (const auto& node : path)
            {
                std::cerr << "---> " << node.second << " " << node.first << " " ;
            }
            std::cerr << "\n";
        }

    }
    else
    {
        std::cerr << "Path does not exits from " << argv[1] << " to " << argv[2] << "\n";
    }



return 0;
}
