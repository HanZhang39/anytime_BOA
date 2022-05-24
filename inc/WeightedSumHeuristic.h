#pragma once

#include "Utils/Definitions.h"

// Precalculates heuristic based on Dijkstra shortest paths algorithm.
// On call to operator() returns the value of the heuristic in O(1)
class WeightedSumHeuristic {

protected:
    size_t                  source;
    double factor;
    std::vector<double>    h_table;

    void compute(const AdjacencyMatrix& adj_matrix);
public:
    clock_t heuristic_time = 0;
    inline double get_factor(){return factor;}
    // return the shortest path of costs cost0 + factor * cost1
    WeightedSumHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix, double factor);
    virtual double operator()(size_t node_id);


};



using WeightedSumHeuristicPtr = std::shared_ptr<WeightedSumHeuristic>;
