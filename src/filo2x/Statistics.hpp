#ifndef _FILO2X_STATISTICS_HPP_
#define _FILO2X_STATISTICS_HPP_

#include <cstdint>
#include <fstream>

#include "filo2/base/StringUtils.hpp"
#include "filo2/base/Timer.hpp"
#include "filo2/base/Welford.hpp"
#include "filo2x/Parameters.hpp"

// Statistics related to main processing.
struct MainStats {
    // Instance initialization time (ms).
    uint64_t inst_init_time = 0;

    // Savings algorithm time (ms).
    uint64_t cw_time = 0;

    // Greedy bin packing estimate (ms).
    uint64_t bpp_time = 0;

    // Overall algorithm time (ms).
    uint64_t tot_time = 0;

    void save(const Parameters& params) const {
        auto out = std::ofstream(params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" +
                                 std::to_string(params.get_seed()) + ".main.stats");
        out << std::setprecision(10);
        out << "inst_init_time=" << inst_init_time << "\n";
        out << "cw_time=" << cw_time << "\n";
        out << "bpp_time=" << bpp_time << "\n";
        out << "tot_time=" << tot_time << "\n";
    }
};

// Statistics related to optimization procedures.
struct OptStats {

    // Time in microseconds required to setup data structures before the optimization.
    uint64_t data_structures_setup_time = 0;
    cobra::Timer data_structures_setup_timer;

    // Total number of received changes that have been evaluated.
    uint64_t rcv_eval_num = 0;

    // Number of discarted changes among the received ones.
    uint64_t rcv_fail_num = 0;

    // Total number of generated changes.
    uint64_t gen_num = 0;

    // Number of generated changes that have been sent.
    uint64_t sent_num = 0;

    // Keeps track of the average queue size at the beginning of each iteration.
    cobra::Welford queue_size;

    // Keeps track of the average number of feasible moves applied per iteration.
    cobra::Welford applied_changes_per_sync;
    uint64_t feasible_per_sync = 0;

    // Total Time in microseconds spent during the procedure.
    uint64_t tot_time = 0;

    // Time in microseconds spent evaluating the feasibility of a received change.
    uint64_t rcv_eval_time = 0;
    cobra::Timer rcv_eval_timer;

    // Time in microseconds spent during the sync phase.
    uint64_t sync_time = 0;
    cobra::Timer sync_time_timer;

    // Time in microseconds spent during the generation phase.
    uint64_t gen_time = 0;
    cobra::Timer gen_time_timer;

    // Time in microseconds spent evaluating the feasibility of a change before sending it.
    uint64_t gen_eval_time = 0;
    cobra::Timer gen_eval_timer;

    // Time in microseconds spent during the ruin step.
    uint64_t gen_ruin_time = 0;
    cobra::Timer gen_ruin_timer;

    // Time in microseconds spent during the recreate step.
    uint64_t gen_recreate_time = 0;
    cobra::Timer gen_recreate_timer;

    // Time in microseconds spent during the ls step.
    uint64_t gen_ls_time = 0;
    cobra::Timer gen_ls_timer;

    // Time in microseconds spent sending the change.
    uint64_t gen_offer_time = 0;
    cobra::Timer gen_offer_timer;

    void save(const Parameters& params, const std::string& procedure, int solver_id) const {
        auto out = std::ofstream(params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" +
                                 std::to_string(params.get_seed()) + ".solver." + std::to_string(solver_id) + "." + procedure + ".stats");
        out << std::setprecision(10);
        out << "data_structures_setup_time=" << data_structures_setup_time << "\n";
        out << "rcv_eval_num=" << rcv_eval_num << "\n";
        out << "rcv_fail_num=" << rcv_fail_num << "\n";
        out << "gen_num=" << gen_num << "\n";
        out << "sent_num=" << sent_num << "\n";
        out << "mean_queue_size_per_iter=" << queue_size.get_mean() << "\n";
        out << "mean_feasible_per_sync=" << applied_changes_per_sync.get_mean() << "\n";
        out << "tot_time=" << tot_time << "\n";
        out << "rcv_eval_time=" << rcv_eval_time << "\n";
        out << "sync_time=" << sync_time << "\n";
        out << "gen_time=" << gen_time << "\n";
        out << "gen_eval_time=" << gen_eval_time << "\n";
        out << "gen_ruin_time=" << gen_ruin_time << "\n";
        out << "gen_recreate_time=" << gen_recreate_time << "\n";
        out << "gen_ls_time=" << gen_ls_time << "\n";
        out << "gen_offer_time=" << gen_offer_time << "\n";
    }
};

#endif