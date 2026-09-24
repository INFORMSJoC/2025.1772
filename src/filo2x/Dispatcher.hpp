#ifndef _FILO2X_DISPATCHER_HPP_
#define _FILO2X_DISPATCHER_HPP_

#include <memory>
#include <thread>

#include "Message.hpp"
#include "Parameters.hpp"
#include "Solver.hpp"
#include "filo2/base/NonCopyable.hpp"
#include "filo2/instance/Instance.hpp"
#include "filo2/solution/Solution.hpp"
#include "filo2x/SharedQueue.hpp"

// Dispatcher thread. Simply relays messages to solvers.
class Dispatcher : public cobra::NonCopyable<Dispatcher> {
public:
    Dispatcher(const cobra::Instance &instance, const Parameters &params, cobra::Solution &solution, int kmin);

    // Make the caller thread wait for this thread to finish before proceeding.
    void join();

    // Offers a candidate coreopt message to the dispatcher.
    void offer_coreopt_msg(const Message &msg);

    // Returns a reference to a solver (id = 0) best solution.
    // This should only be called once the run is finished.
    const cobra::Solution &get_best_solution() const {
        return solvers.front()->get_best_solution();
    }

private:
    // Thread main code.
    void loop();

    // Performs a parallel version of the core optimization procedure.
    void parallel_coreopt();

    // Sends the message to all solvers coreopt queue.
    void broadcast_coreopt(const Message &msg) const;

    // The instance.
    const cobra::Instance &instance;

    // Command line parameters.
    const Parameters &params;

    // Initial solution.
    const cobra::Solution &solution;

    // Estimated minimum number of routes required to solve the instance.
    int kmin;

    // List of optimization solvers.
    std::vector<std::unique_ptr<Solver>> solvers;

    // Thread object.
    std::thread thread;

    // Candidate coreopt changes organized into a min heap.
    SharedQueue<Message> coreopt_queue;
};

#endif