#ifndef BI_CRITERIA_BOA_STAR_H
#define BI_CRITERIA_BOA_STAR_H

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "AbstractSolver.h"


class BOAStar: public AbstractSolver {
protected:
    std::clock_t start_time;

    std::vector<std::pair<std::clock_t, NodePtr>> solution_log;
    void log_solution(NodePtr);

    void end_logging(SolutionSet &solutions, bool succ=true) override;

public:
    virtual std::string get_solver_name() {return "BOA*"; }

    void set_start_time(clock_t t){
        start_time = t;
    };

    BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger=nullptr);

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX);

    std::vector<std::pair<std::clock_t, NodePtr>> get_sol_log(){return solution_log;}
};


class BOAStarRestart: public BOAStar {
protected:

public:
    BOAStarRestart(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger=nullptr);
    void set_eps(Pair<double> new_eps){
        eps = {new_eps[0], new_eps[1] };
    }

    bool is_approximate = false;
    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
    friend class AnytimeBOANaive;
};



class BOAStarBP:public BOAStar {
private:
    const AdjacencyMatrix   &adj_matrix_r;

public:
    BOAStarBP(const AdjacencyMatrix &adj_matrix, const AdjacencyMatrix &adj_matrix_r, Pair<double> eps, const LoggerPtr logger=nullptr);
    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;

};



#endif //BI_CRITERIA_BOA_STAR_H
