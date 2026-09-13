#include "Solver.hpp"

#include "Dispatcher.hpp"
#include "filo2/base/SparseIntSet.hpp"
#include "filo2/base/StringUtils.hpp"
#include "filo2/base/Timer.hpp"
#include "filo2/movegen/MoveGenerators.hpp"
#include "filo2/opt/routemin.hpp"

Solver::Solver(int id, const cobra::Instance& instance, const cobra::Solution& solution, int kmin, const Parameters& params,
               Dispatcher& dispatcher, int shared_seed, int unique_seed)
    : id(id)
    , instance(instance)
    , solution(solution)
    , best_solution(solution)
    , kmin(kmin)
    , params(params)
    , dispatcher(dispatcher)
    , srnd(shared_seed)
    , urnd(unique_seed)
    , thread(&Solver::loop, this) { }

void Solver::join() {
    thread.join();
}

void Solver::send_to_coreopt_queue(const Message& msg) {
    coreopt_queue.push(msg);
}

bool Solver::is_change_applicable(cobra::Solution& solution, const Message& msg, cobra::SparseIntSet& affected_routes) const {
    assert(solution.is_feasible());
    assert(affected_routes.empty());

    // Ruin changes.
    for (const cobra::Solution::Action& action : msg.ruin) {
        if (!solution.is_applicable(action)) {
            return false;
        }
        const int route = solution.apply</*record=*/true>(action);
        assert(route != cobra::Solution::dummy_route);
        affected_routes.insert(route);
    }

    // Recreate changes.
    for (const cobra::Solution::Action& action : msg.recreate) {
        if (!solution.is_applicable(action)) {
            return false;
        }
        const int route = solution.apply</*record=*/true>(action);
        assert(route != cobra::Solution::dummy_route);
        affected_routes.insert(route);
    }

    // Local search changes.
    for (const cobra::Solution::Action& action : msg.localsearch) {
        if (!solution.is_applicable(action)) {
            return false;
        }
        const int route = solution.apply</*record=*/true>(action);
        assert(route != cobra::Solution::dummy_route);
        affected_routes.insert(route);
    }

    // Checks that routes are load-feasible.
    for (int route : affected_routes.get_elements()) {
        if (!solution.is_load_feasible(route)) {
            return false;
        }
    }

    return true;
}

void Solver::output_statistics(const std::string& procedure, uint64_t data_structures_setup_time, uint64_t rcv_eval_num,
                               uint64_t rcv_fail_num, uint64_t gen_num, uint64_t sent_num, const cobra::Welford& queue_size,
                               const cobra::Welford& feasible_per_sync, uint64_t tot_time, uint64_t rcv_eval_time, uint64_t sync_time,
                               uint64_t gen_time, uint64_t gen_eval_time, uint64_t gen_ruin_time, uint64_t gen_recreate_time,
                               uint64_t gen_ls_time) const {

    auto out = std::ofstream(params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" +
                             std::to_string(params.get_seed()) + ".solver." + std::to_string(id) + "." + procedure + ".stats");
    out << std::setprecision(10);
    out << "data_structures_setup_time=" << data_structures_setup_time << "\n";
    out << "rcv_eval_num=" << rcv_eval_num << "\n";
    out << "rcv_fail_num=" << rcv_fail_num << "\n";
    out << "gen_num=" << gen_num << "\n";
    out << "sent_num=" << sent_num << "\n";
    out << "mean_queue_size_per_iter=" << queue_size.get_mean() << "\n";
    out << "mean_feasible_per_sync=" << feasible_per_sync.get_mean() << "\n";
    out << "tot_time=" << tot_time << "\n";
    out << "rcv_eval_time=" << rcv_eval_time << "\n";
    out << "sync_time=" << sync_time << "\n";
    out << "gen_time=" << gen_time << "\n";
    out << "gen_eval_time=" << gen_eval_time << "\n";
    out << "gen_ruin_time=" << gen_ruin_time << "\n";
    out << "gen_recreate_time=" << gen_recreate_time << "\n";
    out << "gen_ls_time=" << gen_ls_time << "\n";
}

void Solver::loop() {

#ifdef VERBOSE
    cobra::Timer timer;
#endif

    uint64_t move_gen_init_time = 0;
    cobra::Timer move_gen_init_timer;

    // Setup move generators data structures.
    auto k = params.get_sparsification_rule_neighbors();
    auto move_generators = cobra::MoveGenerators(instance, k);

    move_gen_init_time += move_gen_init_timer.elapsed_time<std::chrono::nanoseconds>();


    auto out = std::ofstream(params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" +
                             std::to_string(params.get_seed()) + ".solver." + std::to_string(id) + ".setup.stats");
    out << std::setprecision(10);
    out << "move_gen_init_time=" << move_gen_init_time << "\n";

    if (kmin < solution.get_routes_num()) {
#ifdef VERBOSE
        if (id == 0) {
            std::cout << "Running ROUTEMIN heuristic for at most " << params.get_routemin_iterations() << " iterations.\n";
            std::cout << "Starting solution: obj = " << best_solution.get_cost() << ", n. of routes = " << best_solution.get_routes_num()
                      << ".\n";
            timer.reset();
        }
#endif
        assert(best_solution == solution);
        best_solution = routemin(instance, solution, srnd, move_generators, kmin, params.get_routemin_iterations(), params.get_tolerance());
#ifdef VERBOSE
        if (id == 0) {
            std::cout << "Final solution: obj = " << best_solution.get_cost() << ", n. routes = " << best_solution.get_routes_num() << "\n";
            std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n\n";
        }
#endif
    }

    parallel_coreopt(move_generators);
}