#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "ShortestPathHeuristic.h"
#include "WeightedSumHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "BOAStar.h"
#include "anytime_boa.h"
#include "IterativePPA.h"
#include "PPA.h"
#include "SingleCriteria.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

const std::string output_path = "";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;
std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit, vector<double>& lhs, double d) {
    // Compute heuristic
    std::cout << "Start Computing Heuristic" << std::endl;
    auto heuristic_start_time = std::clock();
    std::clock_t heuristic_time;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    heuristic_time = (std::clock() - heuristic_start_time ) ;
    std::cout << "Finish Computing Heuristic: " << (double) heuristic_time / CLOCKS_PER_SEC << "\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int num_exp, num_gen;
    auto runtime = std::clock();
    std::vector<WeightedSumHeuristicPtr> lh_vec;
    heuristic_time = 0; 

    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "anytime"){
        // Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<AnytimeBOA>(graph, d, logger);
        for (auto w: lhs){
            std::cout << "lh with weight " << w << endl;
            auto lh_start_time = std::clock();
            lh_vec.emplace_back(std::make_shared<WeightedSumHeuristic>(target, graph.size(), graph, w));
            heuristic_time += std::clock() - lh_start_time;
            // ((AnytimeBOA*)solver.get())->llh.emplace_back(target, graph.size(), graph, d);
        }
        ((AnytimeBOA*) solver.get()) -> set_lh(& lh_vec);
    }else if (algorithm == "iterative_ppa"){
        // Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<IterativePPA>(graph, logger);
        ((IterativePPA*) solver.get())->set_eps({0.01, 0.001, 0.0001, 0});
    }else if (algorithm == "anytime_naive"){
        // Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<AnytimeBOANaive>(graph, d, logger);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }
    if (logger != nullptr){
        logger->heuristic_time = heuristic_time;
    }
    auto start =std::clock();
    (*solver)(source, target, heuristic, solutions, time_limit);
    runtime = std::clock() - start;

    std::cout << "Heuristic time: " <<  ((double) heuristic_time) / CLOCKS_PER_SEC<< std::endl;

    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();
    int i = 0;
    for (auto sol: solutions){
        std::cout << "sol " << ++i << ": "<< *sol << std::endl;
    }


    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit, vector<double>& lhs, double d) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, lhs, d);
 }

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit, vector<double>& lhs, double d=4) {
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);


    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, lhs, d);
    }

}

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;
    std::vector<double> wsh_list;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "number of agents")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
        ("wsh,w",po::value< std::vector<double> >(&wsh_list)->multitoken(), "files for edge weight")
        ("eps,e", po::value<double>()->default_value(0), "solvers (BOA, PPA, PPA2, Apex, anytime_naive, PPA C)")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("logfile,l", po::value<std::string>()->default_value(""), "logging file" )
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("decrease,d", po::value<double>()->default_value(4), "speed of decreasing")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    if (vm["query"].as<std::string>() != ""){
        if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
            std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
            return -1;
        }
    }
    
    LoggerPtr logger = nullptr;
    auto str = vm["logfile"].as<std::string>();

    if (vm["logfile"].as<std::string>() != ""){
        cout << "init logger " <<  vm["logfile"].as<std::string>()<< std::endl;
        logger = new Logger(vm["logfile"].as<std::string>());
    }

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }


    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
    alg_variant = vm["merge"].as<std::string>();

    if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>()!= "Apex"){
        alg_variant = "";
        std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
        ms = MergeStrategy::SMALLER_G2;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
        ms = MergeStrategy::SMALLER_G2_FIRST;
    }else if(vm["merge"].as<std::string>() == "RANDOM"){
        ms = MergeStrategy::RANDOM;
    }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
        ms = MergeStrategy::MORE_SLACK;
    }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
        ms = MergeStrategy::REVERSE_LEX;
    }else{
        if(vm["merge"].as<std::string>() != ""){
            std::cerr << "unknown merge strategy: " << vm["merge"].as<std::string>()<< std::endl;
        }
    }


    if (vm["query"].as<std::string>() != ""){
        run_query(graph_size, edges, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), wsh_list, vm["decrease"].as<double>());
    } else{
        single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), wsh_list, vm["decrease"].as<double>());
    }

    delete(logger);

    return 0;
}
