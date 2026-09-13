#ifndef _FILO2X_MESSAGE_HPP_
#define _FILO2X_MESSAGE_HPP_

#include <vector>

#include "filo2/solution/Solution.hpp"

struct Message {
    // Ruin actions.
    std::vector<cobra::Solution::Action> ruin;
    // Recreate actions.
    std::vector<cobra::Solution::Action> recreate;
    // Local search actions.
    std::vector<cobra::Solution::Action> localsearch;

    // Value of omega[i] for the ruin walk seed i.
    int walk_seed_omega_value;

    double solution_cost;
    double shaking_ub_factor;
    double shaking_lb_factor;
    double reference_solution_cost;

    // True if the solver communicates that it has finished processing.
    // When this is true, all the other fields have undefined values.
    bool finished = false;
};

#endif