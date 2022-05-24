#include "Utils/Definitions.h"
#include "Utils/MapQueue.h"
#include "AbstractSolver.h"
#include "PPA.h"

class IterativePPA: public PPA {


protected:
    // double d;
    std::vector<double> eps_list;
    void init_search() override;

public:
    virtual std::string get_solver_name() override {return "repeat_PPA"; }

    

    IterativePPA(const AdjacencyMatrix &adj_matrix, const LoggerPtr logger=nullptr):
        PPA(adj_matrix, {}, logger) {}
    ;

    void set_eps(const std::vector<double> input_eps_list){
        eps_list = input_eps_list;
    };

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
};

