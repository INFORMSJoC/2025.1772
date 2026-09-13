#include <chrono>
#include <fstream>

#include "Dispatcher.hpp"
#include "Parameters.hpp"
#include "filo2/base/StringUtils.hpp"
#include "filo2/base/Timer.hpp"
#include "filo2/instance/Instance.hpp"
#include "filo2/opt/bpp.hpp"
#include "filo2/solution/Solution.hpp"
#include "filo2/solution/savings.hpp"
#include "filo2x/Statistics.hpp"

int main(int argc, char* argv[]) {
#ifndef NDEBUG
    std::cout << "******************************\n";
    std::cout << "Probably running in DEBUG mode\n";
    std::cout << "******************************\n\n";
#endif

    cobra::Timer global_timer;
#ifdef VERBOSE
    cobra::Timer timer;
#endif

    const auto params = Parameters(argc, argv);

    std::filesystem::create_directories(params.get_outpath());

    MainStats stats;

    cobra::Timer inst_init_timer;

#ifdef VERBOSE
    std::cout << "Pre-processing the instance.\n";
    timer.reset();
#endif
    std::optional<cobra::Instance> maybe_instance = cobra::Instance::make(params.get_instance_path(), params.get_neighbors_num(),
                                                                          params.get_solvers_num());
#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n\n";
#endif

    if (!maybe_instance.has_value()) {
        return EXIT_FAILURE;
    }

    const cobra::Instance instance = std::move(maybe_instance.value());

    stats.inst_init_time = inst_init_timer.elapsed_time<std::chrono::nanoseconds>();

    auto solution = cobra::Solution(instance, std::min(instance.get_vertices_num(), params.get_solution_cache_size()));

#ifdef VERBOSE
    std::cout << "Running CLARKE&WRIGHT to generate an initial solution.\n";
    timer.reset();
#endif

    cobra::Timer cw_timer;

    cobra::clarke_and_wright(instance, solution, params.get_cw_lambda(), params.get_cw_neighbors());

    stats.cw_time = cw_timer.elapsed_time<std::chrono::nanoseconds>();

#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n";
    std::cout << "Initial solution: obj = " << solution.get_cost() << ", n. of routes = " << solution.get_routes_num() << ".\n\n";

    std::cout << "Computing a greedy upper bound on the n. of routes.\n";
    timer.reset();
#endif

    cobra::Timer bpp_timer;

    auto kmin = bpp::greedy_first_fit_decreasing(instance);

    stats.bpp_time = bpp_timer.elapsed_time<std::chrono::nanoseconds>();

#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::milliseconds>() << " milliseconds.\n";
    std::cout << "Around " << kmin << " routes should do the job.\n\n";

    std::cout << "Spawning " << params.get_solvers_num() << " solver(s) ... (this may take a while)\n\n";
#endif


    Dispatcher dispatcher(instance, params, solution, kmin);
    dispatcher.join();

    const cobra::Solution& best_solution = dispatcher.get_best_solution();

    const int global_time_elapsed = global_timer.elapsed_time<std::chrono::seconds>();

    stats.tot_time = global_timer.elapsed_time<std::chrono::nanoseconds>();

    stats.save(params);

#ifdef VERBOSE
    std::cout << "\n";
    std::cout << "Best solution found:\n";
    std::cout << "obj = " << best_solution.get_cost() << ", n. routes = " << best_solution.get_routes_num() << "\n";

    std::cout << "\n";
    std::cout << "Run completed in " << global_time_elapsed << " seconds ";
#endif

    const auto outfile = params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) +
                         ".out";

    auto out_stream = std::ofstream(outfile);
    out_stream << std::setprecision(10);
    out_stream << best_solution.get_cost() << "\t" << global_time_elapsed << "\n";
    cobra::Solution::store_to_file(
        instance, best_solution,
        params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) + ".vrp.sol");

#ifdef VERBOSE
    std::cout << "\n";
    std::cout << "Results stored in\n";
    std::cout << " - " << outfile << "\n";
    std::cout << " - "
              << params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) + ".vrp.sol"
              << "\n";
#endif

    return EXIT_SUCCESS;
}