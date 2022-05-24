#include "BOAStarContinuing.h"


BOAStarContinuing::BOAStarContinuing(const AdjacencyMatrix &adj_matrix, double eps, const LoggerPtr logger):
    BOAStar(adj_matrix, {eps, eps}, logger) {}


void BOAStarContinuing::operator()(Interval source, size_t target, Heuristic &heuristic, IntervalList& solutions, std::vector<WeightedSumHeuristicPtr>* l_heuristics, unsigned int time_limit) {

    auto start_time = std::clock();

    // this->start_logging(source.to_expand->size(), target);

    // Saving all the unused NodePtrs in a vector improves performace for some reason

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);
    solutions.clear();

    min_g2[target] = source.top_left->f[1];
    size_t max_f1 = source.bottom_right->f[0];

    // Init open heap
    // heap_open_t open;
    std::vector<NodePtr> open;
    Node::more_than_full_cost more_than;
    std::shared_ptr<std::list<NodePtr>> not_expanded = std::make_shared<std::list<NodePtr>>();
    NodePtr prev = source.top_left;

    for (auto & node:*source.to_expand){
        if (l_heuristics != nullptr){
            bool flag = false;
            for (int i = 0; i < l_heuristics->size(); i++){
                // assert(l_heuristics->at(i)(node->id) == ll_heuristics->at(i)(node->id));
                auto h_val = (*(l_heuristics->at(i)))(node->id);
                if (node->g[0] + h_val - l_heuristics->at(i)->get_factor() * (min_g2[target] - node->g[1]) >= max_f1 ){
                    flag = true;
                }
            }
            if (!flag){
                open.push_back(node);
            }
        } else{
            open.push_back(node);
        }
    }
    std::make_heap(open.begin(), open.end(), more_than);

    bool flag = false;
    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            Interval intval(prev, source.bottom_right, not_expanded);
            solutions.push_back(intval);
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        NodePtr node = open.back();
        open.pop_back();
        // std::cout << node->f[0] << std::endl;
        num_generation +=1;

        // Dominance check
        if (node->f[0] >= max_f1){
            continue;
        }

        // flag = false;
        // for (auto & lch: l_heuristics){
        //     if (node->g[0] + lch(node->id) - lch.get_factor() * (min_g2[target] - node->g[1]) >= max_f1 ){
        //         flag = true;
        //     }
        // }
        // if (flag){
        //     continue;
        // }

        if ((((1+this->eps[1])*node->f[1]) >= min_g2[target]) ||
            (node->g[1] >= min_g2[node->id])) {
            if (node->g[1] < min_g2[node->id] && node->f[1] < min_g2[target]){
                not_expanded->push_back(node);
                min_g2[node->id] = node->g[1];
            }
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;

        if (node->id == target) {
            log_solution(node);
            Interval intval(prev, node, not_expanded);
            solutions.push_back(intval);
            prev = node;
            not_expanded = std::make_shared<std::list<NodePtr>>() ;
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            std::vector<size_t> next_h = heuristic(next_id);

            // Dominance check
            if (next_g[0]+next_h[0] >= max_f1){
                continue;
            }
            if (l_heuristics != nullptr){
                flag = false;
                for (int i = 0; i < l_heuristics->size(); i++){
                    // assert(l_heuristics->at(i)(next_id) == ll_heuristics->at(i)(next_id));

                    auto h_val = (*(l_heuristics->at(i)))(next_id);
                    if (next_g[0] + h_val - l_heuristics->at(i)->get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
                // for (auto & lch: *l_heuristics){
                //     if (next_g[0] + lch(next_id) - lch.get_factor() * (min_g2[target] - next_g[1]) >= max_f1 ){
                        flag = true;
                        break;
                    }
                }
                if (flag){
                    continue;
                }
            }


            if ((((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2[target]) ||
                (next_g[1] >= min_g2[next_id])) {
                if (next_g[1] < min_g2[next_id] && (next_g[1]+next_h[1]) < min_g2[target]){
                    not_expanded->push_back(std::make_shared<Node>(next_id, next_g, next_h, node));
                }
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            auto next = std::make_shared<Node>(next_id, next_g, next_h, node);

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

        }
    }

    Interval intval(prev, source.bottom_right, not_expanded);
    solutions.push_back(intval);


    // this->end_logging({});
}
