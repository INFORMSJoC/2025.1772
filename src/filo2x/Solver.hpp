#ifndef _FILO2X_SOLVER_HPP_
#define _FILO2X_SOLVER_HPP_

#include <random>
#include <thread>

#include "Message.hpp"
#include "Parameters.hpp"
#include "SharedQueue.hpp"
#include "filo2/base/NonCopyable.hpp"
#include "filo2/base/SparseIntSet.hpp"
#include "filo2/base/Welford.hpp"
#include "filo2/instance/Instance.hpp"
#include "filo2/movegen/MoveGenerators.hpp"

// Forward declaration.
class Dispatcher;

// Solver thread. Performs the actual optimization.
class Solver : public cobra::NonCopyable<Solver> {
public:
    Solver(int id, const cobra::Instance& instance, const cobra::Solution& solution, int kmin, const Parameters& params,
           Dispatcher& dispatcher, int shared_seed, int unique_seed);

    // Make the caller thread wait for this thread to finish before proceeding.
    void join();

    void send_to_coreopt_queue(const Message& mgs);

    const cobra::Solution& get_best_solution() const {
        return best_solution;
    }

private:
    // Thread main code.
    void loop();

    // Paralle routemin.
    void parallel_routemin(cobra::MoveGenerators& move_generators);

    // Parallel coreopt.
    void parallel_coreopt(cobra::MoveGenerators& move_generators);

    // Checks whether the given change is applicable to solution.
    bool is_change_applicable(cobra::Solution& solution, const Message& msg, cobra::SparseIntSet& affected_routes) const;

    // Stores some thread specific statistics to output files.
    void output_statistics(const std::string& procedure, uint64_t data_structures_setup_time, uint64_t rcv_eval_num, uint64_t rcv_fail_num,
                           uint64_t gen_num, uint64_t sent_num, const cobra::Welford& queue_size, const cobra::Welford& feasible_per_sync,
                           uint64_t tot_time, uint64_t rcv_eval_time, uint64_t sync_time, uint64_t gen_time, uint64_t gen_eval_time,
                           uint64_t gen_ruin_time, uint64_t gen_recreate_time, uint64_t gen_ls_time) const;

    // Thread id.
    int id;

    // The instance.
    const cobra::Instance& instance;

    // Reference solution.
    cobra::Solution solution;

    // Best solution.
    cobra::Solution best_solution;

    // Estimated minimum number of routes required to solve the instance.
    int kmin;

    // Command line parameters.
    const Parameters& params;

    // Reference to the dispatcher object.
    Dispatcher& dispatcher;

    // Pseudo random generator initialized with the same seed for all solvers.
    std::mt19937 srnd;

    // Pseudo random generator initialized with a unique seed.
    std::mt19937 urnd;

    // Thread object.
    std::thread thread;

    // Queues of messages sent by the dispatcher.
    SharedQueue<Message> coreopt_queue;
};

#endif