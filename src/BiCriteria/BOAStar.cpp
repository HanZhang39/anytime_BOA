#include <memory>
#include <algorithm>
#include <time.h>

#include "BOAStar.h"

BOAStar::BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, {eps[0], eps[1]}, logger) {}

void BOAStar::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {
    // int time_limit = 300;
    start_time = std::clock();
    this->start_logging(source, target);

    NodePtr node;
    NodePtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);

    // Init open heap
    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){

            this->end_logging(solutions, false);
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if ((((1+this->eps[1])*node->f[1]) >= min_g2[target]) ||
            (node->g[1] >= min_g2[node->id])) {
            closed.push_back(node);
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;


        if (node->id == target) {
            solutions.push_back(node);
            log_solution(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            auto next_h = heuristic(next_id);

            // Dominance check
            if ((((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2[target]) ||
                (next_g[1] >= min_g2[next_id])) {
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = std::make_shared<Node>(next_id, next_g, next_h, node);

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

            closed.push_back(node);
        }
    }

    this->end_logging(solutions);
}


void BOAStar::end_logging(SolutionSet &solutions, bool succ) {
    // All logging is done in JSON format
    std::stringstream finish_info_json;
    finish_info_json
        << "{\n"
        <<      "\t\"solutions\": [";

    size_t solutions_count = 0;
    for (auto time_solution = solution_log.begin(); time_solution != solution_log.end(); ++time_solution) {

        if (time_solution != solution_log.begin()) {
            finish_info_json << ",";
        }
        auto solution = (*time_solution).second;
        finish_info_json << "\n\t\t" << *time_solution;
        solutions_count++;
    }

    finish_info_json
        <<      "\n\t],\n"
        <<      "\t\"node_expansion\": " << num_expansion << ",\n"
        <<      "\t\"node_generation\": " << num_generation << ",\n"
        <<      "\t\"amount_of_solutions\": " << solutions_count << ",\n"
        <<      "\t\"status\": " << ( succ ? "\"Success\"": "\"Failed\"" )<< "\n"
        << "}" <<std::endl;

    if (this->logger != nullptr) {
        LOG_FINISH_SEARCH(*(this->logger), finish_info_json.str());
    }
}


inline bool is_dominated(SolutionSet & solutions, NodePtr node, Pair<double> eps = {0,0}){
    for (auto sol: solutions){
        if (sol->g[0] <= (1 + eps[0]) * node->g[0] + node->h[0] &&
            sol->g[1] <= (1 + eps[1]) * node->g[1] + node->h[1]
            ){
            return true;
        }
    }
    return false;
}


BOAStarRestart::BOAStarRestart(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger): BOAStar(adj_matrix,eps){
}
 

void BOAStarRestart::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {
    start_time = std::clock();
    this->start_logging(source, target);


    is_approximate = false;
    NodePtr node;
    NodePtr next;

    SolutionSet new_solutions;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<size_t> min_g2(this->adj_matrix.size()+1, MAX_COST);

    // Init open heap
    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            this->end_logging(solutions, false);
            // std::cout << "BOA fails" << std::endl;
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if (node->g[1] >= min_g2[node->id] || node->f[1] >= min_g2[target] || is_dominated(solutions, node) ){
            continue;
        }

        if (((1+this->eps[1])*node->f[1]) >= min_g2[target] || is_dominated(solutions, node, {eps[0], eps[1]})){
            closed.push_back(node);
            is_approximate = true;
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;


        if (node->id == target) {
            new_solutions.push_back(node);
            log_solution(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            auto next_h = heuristic(next_id);

            next = std::make_shared<Node>(next_id, next_g, next_h, node);

            if  (next_g[1] >= min_g2[next_id] || next_g[1] + next_h[1] >= min_g2[target] || is_dominated(solutions, next)) {
                continue;
            }

            // Dominance check
            if (((1+this->eps[1])*(next_g[1]+next_h[1])) >= min_g2[target] || is_dominated(solutions, next, {eps[0], eps[1]})){
                is_approximate = true;
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);

            closed.push_back(node);
        }
    }

    this->end_logging(solutions);


    // TODO merge two solutions
    SolutionSet tmp = solutions;
    solutions.clear();
    int i_tmp = 0;
    int i_new = 0;
    while(i_tmp < tmp.size() || i_new < new_solutions.size()){
        if (i_tmp >= tmp.size()){
            solutions.push_back(new_solutions[i_new]);
            i_new ++;
            continue;
        }
        if (i_new >= new_solutions.size()){
            solutions.push_back(tmp[i_tmp]);
            i_tmp ++;
            continue;
        }
        if (new_solutions[i_new]->g[0] == tmp[i_tmp]->g[0]){
            solutions.push_back(new_solutions[i_new]);
            i_new ++;
            i_tmp ++;
        } else if(new_solutions[i_new]->g[0] < tmp[i_tmp]->g[0]){
            solutions.push_back(new_solutions[i_new]);
            i_new ++;
        } else {
            solutions.push_back(tmp[i_tmp]);
            i_tmp ++;
        }
    }

}



void BOAStar::log_solution(NodePtr node){
    solution_log.push_back({std::clock() - start_time, node});
}
