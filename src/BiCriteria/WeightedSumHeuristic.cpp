#include <limits>
#include <memory>
#include <algorithm>
#include <float.h>

#include "WeightedSumHeuristic.h"


WeightedSumHeuristic::WeightedSumHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix, double factor)
    : source(source), factor(factor), h_table(graph_size+1, DBL_MAX) {
    // auto start_time = std::clock();
    compute(adj_matrix);
    // heuristic_time += std::clock() - start_time;

}


double WeightedSumHeuristic::operator()(size_t node_id) {
    return this->h_table[node_id];
}


// Implements Dijkstra shortest path algorithm per cost_idx cost function
void WeightedSumHeuristic::compute(const AdjacencyMatrix &adj_matrix) {
    // Init all heuristics to MAX_COST

    NodePtr node;
    // NodePtr next;

    // Init open heap
    Node::more_than_combined_heurisitic more_than(factor);
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    open.push_back(std::make_shared<Node>(this->source, std::vector<size_t>(2,0), std::vector<size_t>(2,0)));
    std::push_heap(open.begin(), open.end(), more_than);

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        if (h_table[node->id] != DBL_MAX){
            continue;
        }

        h_table[node->id] = node->g[0] + this->factor*((double)node->g[1]);

        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // if node is closed, do not generate
            if (this->h_table[p_edge->target] != DBL_MAX){
                continue;
            }

            open.push_back(std::make_shared<Node>(p_edge->target, std::vector<size_t>({node->g[0] + p_edge->cost[0], node->g[1] + p_edge->cost[1]}), std::vector<size_t>({0,0})));
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
}


