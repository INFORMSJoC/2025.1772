#include "Dispatcher.hpp"

#include "Solver.hpp"

Dispatcher::Dispatcher(const cobra::Instance& instance, const Parameters& params, cobra::Solution& solution, int kmin)
    : instance(instance), params(params), solution(solution), kmin(kmin), thread(&Dispatcher::loop, this) { }

void Dispatcher::join() {
    thread.join();
}

void Dispatcher::offer_coreopt_msg(const Message& msg) {
    coreopt_queue.push(msg);
}

void Dispatcher::broadcast_coreopt(const Message& msg) const {
    for (auto& solver : solvers) {
        solver->send_to_coreopt_queue(msg);
    }
}

void Dispatcher::parallel_coreopt() {
    int active_solvers = params.get_solvers_num();
    while (active_solvers) {
        // TODO: consider adding some stats.
        Message msg = coreopt_queue.get();
        if (msg.finished) {
            --active_solvers;
            continue;
        }
        broadcast_coreopt(msg);
    }
}

void Dispatcher::loop() {

    // Create and spawn the solvers.
    for (int i = 0; i < params.get_solvers_num(); ++i) {
        const int shared_seed = params.get_seed();
        const int unique_seed = params.get_seed() + i;
        solvers.push_back(std::make_unique<Solver>(i, instance, solution, kmin, params, *this, shared_seed, unique_seed));
    }

    parallel_coreopt();

    for (auto& solver : solvers) {
        solver->join();
    }
}
