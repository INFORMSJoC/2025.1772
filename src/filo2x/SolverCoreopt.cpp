#include <random>

#include "Dispatcher.hpp"
#include "Solver.hpp"
#include "filo2/base/PrettyPrinter.hpp"
#include "filo2/base/SparseIntSet.hpp"
#include "filo2/base/Timer.hpp"
#include "filo2/base/Welford.hpp"
#include "filo2/localsearch/LocalSearch.hpp"
#include "filo2/movegen/MoveGenerators.hpp"
#include "filo2/opt/RuinAndRecreate.hpp"
#include "filo2/opt/SimulatedAnnealing.hpp"
#include "filo2x/Statistics.hpp"

namespace {
    constexpr char PROCEDURE_NAME[] = "coreopt";
}

void Solver::parallel_coreopt(cobra::MoveGenerators& move_generators) {

    cobra::Timer tot_time_timer;

    assert(solution == best_solution);

    OptStats stats;

    stats.data_structures_setup_timer.reset();

    // Local search data structures setup.
    auto rvnd0 = cobra::RandomizedVariableNeighborhoodDescent</*record=*/true>(
        instance, move_generators,
        {cobra::E11,   cobra::E10,   cobra::TAILS, cobra::SPLIT, cobra::RE22B, cobra::E22,  cobra::RE20,  cobra::RE21,
         cobra::RE22S, cobra::E21,   cobra::E20,   cobra::TWOPT, cobra::RE30,  cobra::E30,  cobra::RE33B, cobra::E33,
         cobra::RE31,  cobra::RE32B, cobra::RE33S, cobra::E31,   cobra::E32,   cobra::RE32S},
        urnd, params.get_tolerance());

    auto rvnd1 = cobra::RandomizedVariableNeighborhoodDescent</*record=*/true>(instance, move_generators, {cobra::EJCH}, urnd,
                                                                               params.get_tolerance());

    auto local_search = cobra::VariableNeighborhoodDescentComposer(params.get_tolerance());
    local_search.append(&rvnd0);
    local_search.append(&rvnd1);


    // Granular neighborhood and sparsification data structures setup.
    const auto gamma_base = params.get_gamma_base();
    auto gamma = std::vector<double>(instance.get_vertices_num(), gamma_base);
    auto gamma_counter = std::vector<int>(instance.get_vertices_num(), 0);

    auto gamma_vertices = std::vector<int>();
    for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
        gamma_vertices.emplace_back(i);
    }
    move_generators.set_active_percentage(gamma, gamma_vertices);


    const auto delta = params.get_delta();
    auto average_number_of_vertices_accessed = cobra::Welford();

    // Cost of the working solution, from which neighbor is obtained after shaking and local search.
    double reference_solution_cost = solution.get_cost();


    // Ruin & recreate data structures setup.
    auto rr = RuinAndRecreate(instance, urnd);
    auto ruined_customers = std::vector<int>();

    const auto omega_base = std::max(1, static_cast<int>(std::ceil(std::log(instance.get_vertices_num()))));
    auto omega = std::vector<int>(instance.get_vertices_num(), omega_base);
    auto random_choice = std::uniform_int_distribution(0, 1);

    const auto intensification_lb = params.get_shaking_lb_factor();
    const auto intensification_ub = params.get_shaking_ub_factor();

    const auto mean_solution_arc_cost = solution.get_cost() / (static_cast<double>(instance.get_customers_num()) +
                                                               2.0 * static_cast<double>(solution.get_routes_num()));

    auto shaking_lb_factor = mean_solution_arc_cost * intensification_lb;
    auto shaking_ub_factor = mean_solution_arc_cost * intensification_ub;

    int coreopt_iterations = params.get_coreopt_iterations();
    int feasible_iter = 0;

    // Simulated annealing setup.
    auto vertices_dist = std::uniform_int_distribution(instance.get_vertices_begin(), instance.get_vertices_end() - 1);
    cobra::Welford sampled_arc_cost;
    for (int i = 0; i < instance.get_vertices_num(); ++i) {
        sampled_arc_cost.update(instance.get_cost(vertices_dist(srnd), vertices_dist(srnd)));
    }

    const auto sa_initial_temperature = sampled_arc_cost.get_mean() * params.get_sa_initial_factor();
    const auto sa_final_temperature = sa_initial_temperature * params.get_sa_final_factor();

    auto sa = cobra::SimulatedAnnealing(sa_initial_temperature, sa_final_temperature, srnd, coreopt_iterations);

#ifdef VERBOSE
    if (id == 0) {
        std::cout << "Simulated annealing temperature goes from " << sa_initial_temperature << " to " << sa_final_temperature << ".\n\n";
    }
#endif

    // Storage used below. Declared here to minimize memory allocations.
    cobra::SparseIntSet affected_routes(instance.get_vertices_num());

    // The candidate change. Declared here to minimize memory allocations.
    Message change;

#ifdef VERBOSE
    if (id == 0) {
        std::cout << "Running COREOPT for " << coreopt_iterations << " iterations.\n";
    }

    auto printer = cobra::PrettyPrinter({{"%", cobra::PrettyPrinter::Field::Type::REAL, 5, " "},
                                         {"Iterations", cobra::PrettyPrinter::Field::Type::INTEGER, 10, " "},
                                         {"Objective", cobra::PrettyPrinter::Field::Type::INTEGER, 10, " "},
                                         {"Routes", cobra::PrettyPrinter::Field::Type::INTEGER, 6, " "},
                                         {"Iter/s", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"Eta (s)", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"Gamma", cobra::PrettyPrinter::Field::Type::REAL, 5, " "},
                                         {"Omega", cobra::PrettyPrinter::Field::Type::REAL, 6, " "},
                                         {"Temp", cobra::PrettyPrinter::Field::Type::REAL, 6, " "}});

    cobra::Timer timer;
    cobra::Timer coreopt_timer;
#endif

    stats.data_structures_setup_time = stats.data_structures_setup_timer.elapsed_time<std::chrono::nanoseconds>();

    // Let's loop until we process coreopt_iteration messages.
    while (true) {
        assert(solution.is_feasible());

#ifdef VERBOSE
        if (id == 0 && coreopt_timer.elapsed_time<std::chrono::seconds>() > 1) {
            coreopt_timer.reset();

            const auto progress = 100.0 * (feasible_iter + 1.0) / coreopt_iterations;
            const auto elapsed_seconds = timer.elapsed_time<std::chrono::seconds>();
            const auto iter_per_second = static_cast<double>(feasible_iter + 1) / (static_cast<double>(elapsed_seconds) + 0.01);
            const auto remaining_iter = coreopt_iterations - feasible_iter;
            const auto estimated_rem_time = static_cast<double>(remaining_iter) / iter_per_second;

            auto gamma_mean = 0.0;
            for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
                gamma_mean += gamma[i];
            }
            gamma_mean = (gamma_mean / static_cast<double>(instance.get_vertices_num()));

            auto omega_mean = 0.0;
            for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {
                omega_mean += omega[i];
            }
            omega_mean /= static_cast<double>(instance.get_customers_num());


            printer.print(progress, feasible_iter + 1, best_solution.get_cost(), best_solution.get_routes_num(), iter_per_second,
                          estimated_rem_time, gamma_mean, omega_mean, sa.get_temperature());
        }
#endif

        stats.sync_time_timer.reset();

        // Go through the available updates and apply the feasible ones.
        // Note that we take a snapshot of the queue size, as we expect to continuously receive messages.
        int current_queue_size = coreopt_queue.size();
        stats.queue_size.update(current_queue_size);
        stats.feasible_per_sync = 0;

        while (current_queue_size-- > 0) {
            // Restore the reference solution.
            solution.apply_undo_list1(solution);
            solution.clear_do_list1();
            solution.clear_undo_list1();
            solution.clear_svc();
            assert(solution.is_feasible());

            const Message msg = coreopt_queue.get();
            stats.rcv_eval_num++;

            // Termination messages should not be broadcast to solvers.
            assert(!msg.finished);

            // We are not expecting empty changes.
            assert(msg.ruin.size() + msg.recreate.size() + msg.localsearch.size() > 0);

            // We expect the first action in the ruin is the removal of the seed customer.
            assert(msg.ruin.front().type == cobra::Solution::ActionType::REMOVE_VERTEX ||
                   msg.ruin.front().type == cobra::Solution::ActionType::REMOVE_ONE_CUSTOMER_ROUTE);

            stats.rcv_eval_timer.reset();

            // Clear the storage containing the routes affected by the current change.
            affected_routes.clear();

            // Ruin changes.
            for (const cobra::Solution::Action& action : msg.ruin) {
                if (!solution.is_applicable(action)) {
                    goto undo_and_continue;
                }
                const int route = solution.apply</*record=*/true>(action);
                assert(route != cobra::Solution::dummy_route);
                affected_routes.insert(route);
            }

            // Recreate changes.
            for (const cobra::Solution::Action& action : msg.recreate) {
                if (!solution.is_applicable(action)) {
                    goto undo_and_continue;
                }
                const int route = solution.apply</*record=*/true>(action);
                assert(route != cobra::Solution::dummy_route);
                affected_routes.insert(route);
            }

            ruined_customers.clear();
            for (auto i = solution.get_svc_begin(); i != solution.get_svc_end(); i = solution.get_svc_next(i)) {
                ruined_customers.emplace_back(i);
            }

            // Local search changes.
            for (const cobra::Solution::Action& action : msg.localsearch) {
                if (!solution.is_applicable(action)) {
                    goto undo_and_continue;
                }
                const int route = solution.apply</*record=*/true>(action);
                assert(route != cobra::Solution::dummy_route);
                affected_routes.insert(route);
            }

            // Check the load feasibity of affected routes.
            for (int route : affected_routes.get_elements()) {
                if (!solution.is_load_feasible(route)) {
                    goto undo_and_continue;
                }
            }

            stats.rcv_eval_time += stats.rcv_eval_timer.elapsed_time<std::chrono::nanoseconds>();

            // If we reach this point we have a feasible solution.
            assert(solution.is_feasible());
            feasible_iter++;
            stats.feasible_per_sync++;

            // Put code into a block as the above gotos bypasses initialization of some local variable.
            {

                average_number_of_vertices_accessed.update(static_cast<double>(solution.get_svc_size()));

                auto max_non_improving_iterations = static_cast<int>(std::ceil(
                    delta * static_cast<double>(coreopt_iterations) * static_cast<double>(average_number_of_vertices_accessed.get_mean()) /
                    static_cast<double>(instance.get_vertices_num())));

                bool improved_best_solution;
                if (solution.get_cost() < best_solution.get_cost()) {
                    improved_best_solution = true;

                    solution.apply_do_list2(best_solution);
                    solution.apply_do_list1(best_solution);  // Latest changes.
                    solution.clear_do_list2();

                    assert(best_solution.is_feasible());
                    assert(best_solution == solution);

                    gamma_vertices.clear();
                    for (auto i = solution.get_svc_begin(); i != solution.get_svc_end(); i = solution.get_svc_next(i)) {
                        gamma[i] = gamma_base;
                        gamma_counter[i] = 0;
                        gamma_vertices.emplace_back(i);
                    }
                    move_generators.set_active_percentage(gamma, gamma_vertices);
                } else {
                    improved_best_solution = false;

                    for (auto i = solution.get_svc_begin(); i != solution.get_svc_end(); i = solution.get_svc_next(i)) {
                        gamma_counter[i]++;
                        if (gamma_counter[i] >= max_non_improving_iterations) {
                            gamma[i] = std::min(gamma[i] * 2.0, 1.0);
                            gamma_counter[i] = 0;
                            gamma_vertices.clear();
                            gamma_vertices.emplace_back(i);
                            move_generators.set_active_percentage(gamma, gamma_vertices);
                        }
                    }
                }

                // Check whether we are done.
                if (feasible_iter == params.get_coreopt_iterations()) {
                    // Send termination message.
                    change.finished = true;
                    dispatcher.offer_coreopt_msg(change);

                    stats.tot_time = tot_time_timer.elapsed_time<std::chrono::nanoseconds>();

                    stats.save(params, PROCEDURE_NAME, id);
                    return;
                }

                // Use the value that was originally used to perform the ruin.
                const auto seed_shake_value = msg.walk_seed_omega_value;

                // Update omega values.
                if (msg.solution_cost > msg.shaking_ub_factor + msg.reference_solution_cost) {

                    for (auto i : ruined_customers) {
                        if (omega[i] > seed_shake_value - 1) {
                            omega[i]--;
                        }
                    }
                } else if (msg.solution_cost >= msg.reference_solution_cost &&
                           msg.solution_cost < msg.reference_solution_cost + msg.shaking_lb_factor) {

                    for (auto i : ruined_customers) {
                        if (omega[i] < seed_shake_value + 1) {
                            omega[i]++;
                        }
                    }
                } else {

                    for (auto i : ruined_customers) {
                        if (random_choice(srnd)) {
                            if (omega[i] > seed_shake_value - 1) {
                                omega[i]--;
                            }
                        } else {
                            if (omega[i] < seed_shake_value + 1) {
                                omega[i]++;
                            }
                        }
                    }
                }


                if (sa.accept(reference_solution_cost, solution)) {
                    if (!improved_best_solution) {
                        solution.append_do_list1_to_do_list2();
                    }

                    solution.clear_do_list1();
                    solution.clear_undo_list1();

                    reference_solution_cost = solution.get_cost();

                    const auto updated_mean_solution_arc_cost = solution.get_cost() /
                                                                (static_cast<double>(instance.get_customers_num()) +
                                                                 2.0 * static_cast<double>(solution.get_routes_num()));
                    shaking_lb_factor = updated_mean_solution_arc_cost * intensification_lb;
                    shaking_ub_factor = updated_mean_solution_arc_cost * intensification_ub;
                }

                sa.decrease_temperature();
            }

            // We use the section below only to undo unfeasible changes. This is required if we are performing the last iteration and we
            // cannot thus rely on the top cleanup.
            continue;

        undo_and_continue:
            solution.apply_undo_list1(solution);
            solution.clear_do_list1();
            solution.clear_undo_list1();
            solution.clear_svc();
            assert(solution.is_feasible());

            stats.rcv_fail_num++;
            stats.rcv_eval_time += stats.rcv_eval_timer.elapsed_time<std::chrono::nanoseconds>();
        }

        stats.applied_changes_per_sync.update(stats.feasible_per_sync);

        // Reset the lists possibly containing the last iteration of updates.
        solution.apply_undo_list1(solution);
        solution.clear_do_list1();
        solution.clear_undo_list1();
        solution.clear_svc();
        assert(solution.is_feasible());

        stats.sync_time += stats.sync_time_timer.elapsed_time<std::chrono::nanoseconds>();

        stats.gen_time_timer.reset();

        // Apply the ruin.
        stats.gen_ruin_timer.reset();

        int walk_seed = rr.ruin</*record=*/true>(solution, omega);
        const int ruin_actions_end = solution.get_do_list1().size();
        stats.gen_ruin_time += stats.gen_ruin_timer.elapsed_time<std::chrono::nanoseconds>();

        // Apply the recreate.
        stats.gen_recreate_timer.reset();
        rr.recreate</*record=*/true>(solution);
        const int recreate_actions_end = solution.get_do_list1().size();
        stats.gen_recreate_time += stats.gen_recreate_timer.elapsed_time<std::chrono::nanoseconds>();

        // Apply the local search.
        stats.gen_ls_timer.reset();
        local_search.sequential_apply(solution);
        assert(solution.is_feasible());
        stats.gen_ls_time += stats.gen_ls_timer.elapsed_time<std::chrono::nanoseconds>();

        // Prepare the change.
        change.ruin.clear();
        change.recreate.clear();
        change.localsearch.clear();

        // Fill the change.
        for (int i = 0; i < ruin_actions_end; ++i) {
            change.ruin.emplace_back(solution.get_do_list1()[i]);
        }
        for (int i = ruin_actions_end; i < recreate_actions_end; ++i) {
            change.recreate.emplace_back(solution.get_do_list1()[i]);
        }
        for (int i = recreate_actions_end; i < static_cast<int>(solution.get_do_list1().size()); ++i) {
            change.localsearch.emplace_back(solution.get_do_list1()[i]);
        }
        change.walk_seed_omega_value = omega[walk_seed];
        change.solution_cost = solution.get_cost();
        change.shaking_lb_factor = shaking_lb_factor;
        change.shaking_ub_factor = shaking_ub_factor;
        change.reference_solution_cost = reference_solution_cost;

        ++stats.gen_num;

        stats.gen_time += stats.gen_time_timer.elapsed_time<std::chrono::nanoseconds>();

        stats.gen_offer_timer.reset();
        dispatcher.offer_coreopt_msg(change);
        stats.gen_offer_time += stats.gen_offer_timer.elapsed_time<std::chrono::nanoseconds>();

        ++stats.sent_num;
    }
}
