#pragma once

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "WeightedSumHeuristic.h"
#include "BOAStar.h".h"

class BOAStarContinuing: public BOAStar {
private:

public:

    virtual std::string get_solver_name() {return "BOA* Continuing"; }

    // std::vector<std::pair<std::clock_t, NodePtr>> solution_log;

    BOAStarContinuing(const AdjacencyMatrix &adj_matrix, double eps, const LoggerPtr logger=nullptr);
    virtual void operator()(Interval source, size_t target, Heuristic &heuristic, IntervalList & solutions, std::vector<WeightedSumHeuristicPtr>* l_heuristics, unsigned int time_limit=UINT_MAX);

    friend class AnytimeBOA;
};
