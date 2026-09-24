#ifndef _FILO2_NEIGHBORACCEPTANCE_HPP_
#define _FILO2_NEIGHBORACCEPTANCE_HPP_

#include <cmath>
#include <random>

#include "filo2/solution/Solution.hpp"

namespace cobra {

    // Simulated annealing helper class.
    class SimulatedAnnealing {
    public:
        SimulatedAnnealing(double initial_temperature_, double final_temperature_, std::mt19937 &rand_engine_, int max_iter_)
            : rand_engine(rand_engine_), uniform_dist(0.0, 1.0) {

            initial_temperature = initial_temperature_;

            temperature = initial_temperature;
            factor = std::pow(final_temperature_ / initial_temperature, 1.0 / static_cast<double>(max_iter_));
        }

        SimulatedAnnealing(double initial_temperature_, double cooldown_factor_, std::mt19937 &rand_engine_)
            : rand_engine(rand_engine_), uniform_dist(0.0, 1.0) {

            initial_temperature = initial_temperature_;
            temperature = initial_temperature;
            factor = cooldown_factor_;
        }

        void decrease_temperature() {
            temperature *= factor;
        }

        bool accept(const double reference_solution_cost, const Solution &neighbor) {
            return neighbor.get_cost() < reference_solution_cost - temperature * std::log(uniform_dist(rand_engine));
        }

        double get_temperature() const {
            return temperature;
        }

    private:
        double initial_temperature;
        double temperature;

        std::mt19937 &rand_engine;
        std::uniform_real_distribution<double> uniform_dist;

        double factor;
    };

}  // namespace cobra

#endif