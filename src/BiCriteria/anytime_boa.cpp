#include "SingleCriteria.h"
#include "anytime_boa.h"

AnytimeBOA::AnytimeBOA(const AdjacencyMatrix &adj_matrix, double d, const LoggerPtr logger):
    AbstractSolver(adj_matrix, {}, logger), d(d) {}


struct interval_comp{
    bool operator()(const Interval n1, const Interval n2) const
    {
        return n1.eps < n2.eps;
    }
};
typedef boost::heap::pairing_heap<Interval, boost::heap::compare<interval_comp> > IntervalQueue;


void AnytimeBOA::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit){
    start_time = std::clock();
    this->start_logging(source, target);

    AStar a_star(adj_matrix);
    auto sol_bottom_up = a_star(source, target, heuristic);
    log_solution(sol_bottom_up);
    auto sol_top_left = a_star(source, target, heuristic, Node::LEX_ORDER::LEX1);
    log_solution(sol_top_left);

    clock_t boa_time = 0;

    auto node = std::make_shared<Node>(source, std::vector<size_t>({0,0}), heuristic(source));

    auto tmp_ptr =std::make_shared<std::list<NodePtr>>();
    tmp_ptr->push_back(node);
    IntervalQueue to_expand;
    to_expand.push(Interval(sol_bottom_up, sol_top_left, tmp_ptr));

    while(true){
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            solutions.clear();
            solutions.reserve(to_expand.size() + 1);
            NodePtr rightmost=nullptr;
            for (auto interval: to_expand){
                if (rightmost == nullptr){
                    rightmost = interval.bottom_right;
                } else if(interval.bottom_right->f[0] > rightmost->f[0]){
                    rightmost = interval.bottom_right;
                }
                solutions.push_back(interval.top_left);
            }
            solutions.push_back(rightmost);
            end_logging(solutions, false);
            return;
        }
 

        auto interval =  to_expand.top();
        if (interval.eps == 0){
            // Found all solutions;
            solutions.clear();
            solutions.reserve(to_expand.size() + 1);
            NodePtr rightmost=nullptr;
            for (auto interval: to_expand){
                if (rightmost == nullptr){
                    rightmost = interval.bottom_right;
                } else if(interval.bottom_right->f[0] > rightmost->f[0]){
                    rightmost = interval.bottom_right;
                }
                solutions.push_back(interval.top_left);
            }
            solutions.push_back(rightmost);
            std::cout << "BOA conti time: " << boa_time/CLOCKS_PER_SEC << " s" << std::endl;
            end_logging(solutions);

            std::clock_t lazy_h_compute_time = 0;
            for (auto it_lh: *lh){
                lazy_h_compute_time += it_lh->heuristic_time;
            }
            std::cout << "  lazy h compute time " << ((double)lazy_h_compute_time)/CLOCKS_PER_SEC<< std::endl;


            return;
        }
        to_expand.pop();

        IntervalList refined_interval;
        auto boa_start = std::clock();
        BOAStarContinuing boac(adj_matrix, interval.eps / d);
        boac.set_start_time(start_time);
        boac(interval, target, heuristic, refined_interval, lh, time_limit -(std::clock() - start_time)/CLOCKS_PER_SEC + 1);
        boa_time += std::clock() - boa_start;

        solution_log.insert(solution_log.end(), boac.solution_log.begin(), boac.solution_log.end());
        num_expansion += boac.get_num_expansion();
        num_generation += boac.get_num_generation();

        for (auto new_interval:refined_interval){
            to_expand.push(new_interval);
        }

        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            continue;
        }

    }
    this->end_logging(solutions);


}




void AnytimeBOA::log_solution(NodePtr node){
    solution_log.push_back({std::clock() - start_time, node});
}

void AnytimeBOA::end_logging(SolutionSet &solutions, bool succ) {
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
