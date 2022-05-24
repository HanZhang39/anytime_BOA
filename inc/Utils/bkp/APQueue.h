#include <set>
#include <vector>
#include <list>
#include "Utils/Definitions.h"

class APQueue
{
private:
    std::vector<ApexPathPairPtr>                heap;
    ApexPathPair::more_than_full_cost           more_than;

    std::vector<std::vector<ApexPathPairPtr>>   open_map;

public:
    APQueue(size_t graph_size);
    bool empty();
    ApexPathPairPtr top();
    ApexPathPairPtr pop();
    void insert(ApexPathPairPtr &pp);
    std::vector<ApexPathPairPtr> &get_open_pps(size_t id);

};
