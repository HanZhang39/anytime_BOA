#pragma once

#include "BOAStarContinuing.h"
#include "AbstractSolver.h"

class AnytimeBOA : public AbstractSolver {
protected:
    double d;

    std::clock_t start_time;
    std::vector<std::pair<std::clock_t, NodePtr>> solution_log;

    std::vector<WeightedSumHeuristicPtr> * lh=nullptr;

    void log_solution(NodePtr);
    void end_logging(SolutionSet &solutions, bool succ=true) override;
    
public:

    virtual std::string get_solver_name()  override {return "Anytime BOA"; }

    AnytimeBOA(const AdjacencyMatrix &adj_matrix, double d = 4, const LoggerPtr logger=nullptr);

    void set_lh(std::vector<WeightedSumHeuristicPtr> * lh_ptr){
        lh = lh_ptr;
    }

    // virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, std::vector<WeightedSumHeuristic>& l_heuristics, unsigned int time_limit=UINT_MAX);

    virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX)  override;
    // {
    //     std::vector<WeightedSumHeuristic> vec;
    //     (*this)(source, target, heuristic, solutions, vec, time_limit);
    // };

    std::vector<std::pair<std::clock_t, NodePtr>> get_sol_log(){return solution_log;}
};

class AnytimeBOANaive: public AnytimeBOA {

public:
    virtual std::string get_solver_name()  override {return "Anytime BOA Naive " + std::to_string(d); }

    AnytimeBOANaive(const AdjacencyMatrix &adj_matrix, double d = 4, const LoggerPtr logger=nullptr):
        AnytimeBOA(adj_matrix, d, logger)
    {}

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
};


