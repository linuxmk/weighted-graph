#include "graph.h"

int main(int argc, char *argv[])
{

    if(argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " startNode endNode\n";
        return 1;
    }

    Graph g("/home/rb/devel/projects/work/epay/weightedGraph/graph.txt");

//    Graph g("graph.txt");

    auto retVal = g.findMaxWeightPath(argv[1], argv[2]);

    if(retVal != -1)
    {
        std::cerr << "PAth exits and its weight is " << retVal << "\n";
    }
    else
    {
        std::cerr << "PAth does not exits from " << argv[1] << " to " << argv[2] << "\n";
    }

return 0;
}
