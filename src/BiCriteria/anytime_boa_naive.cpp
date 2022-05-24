#include "SingleCriteria.h"
#include "anytime_boa.h"

void AnytimeBOANaive::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit){

    solutions.clear();

    start_time = std::clock();
    this->start_logging(source, target);

    AStar a_star(adj_matrix);
    auto sol_top_left = a_star(source, target, heuristic);
    log_solution(sol_top_left);
    auto sol_bottom_right = a_star(source, target, heuristic, Node::LEX_ORDER::LEX1);
    log_solution(sol_bottom_right);

    solutions.push_back(sol_top_left);
    solutions.push_back(sol_bottom_right);

    clock_t boa_time = 0;

    auto node = std::make_shared<Node>(source, std::vector<size_t>({0,0}), heuristic(source));

    // auto tmp_ptr =std::make_shared<std::list<NodePtr>>();
    // tmp_ptr->push_back(node);
    // InervalQueue to_expand;


    double eps = std::min( ((double)sol_top_left->f[1]) / sol_bottom_right->f[1] - 1, ((double)sol_bottom_right->f[0])/sol_top_left->f[0] - 1  )/d;

    BOAStarRestart boa(adj_matrix, {eps, eps}, nullptr);
    boa.set_start_time(start_time);

    while(true){
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            solution_log.insert(solution_log.end(), boa.solution_log.begin(), boa.solution_log.end());
            this->end_logging(solutions, false);
            num_expansion = boa.get_num_expansion();
            num_generation = boa.get_num_generation();
            return;
        }
        std::cout << eps << std::endl;
        boa.set_eps({eps,eps});
        boa(source, target, heuristic, solutions, time_limit -(std::clock() - start_time)/CLOCKS_PER_SEC + 1);

        if ( ! boa.is_approximate){
            solution_log.insert(solution_log.end(), boa.solution_log.begin(), boa.solution_log.end());

            this->end_logging(solutions, false);
            num_expansion = boa.get_num_expansion();
            num_generation = boa.get_num_generation();
            return;
        }

        eps = eps / d;

        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            continue;
        }
    }
}
