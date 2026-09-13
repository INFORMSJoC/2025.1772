#include "Instance.hpp"

#include <algorithm>

#include "filo2/base/KDTree.hpp"
#include "filo2/base/Timer.hpp"
#include "filo2/instance/Parser.hpp"

#ifdef VERBOSE
    #include <iostream>
#endif

namespace cobra {

    // static
    std::optional<Instance> Instance::make(const std::string& filepath, int neighbors_num, int threads_num,
                                           const std::vector<std::vector<int>>& neighbors_) {

        Parser parser(filepath);

        std::optional<InstanceData> maybe_data = parser.Parse();
        if (!maybe_data.has_value()) {
            return std::nullopt;
        }

        return Instance(maybe_data.value(), neighbors_num, threads_num, neighbors_);
    }

    // static
    Instance Instance::make(const InstanceData& data, int num_neighbors, int threads_num, const std::vector<std::vector<int>>& neighbors_) {
        return Instance(data, num_neighbors, threads_num, neighbors_);
    }

    Instance::Instance(const InstanceData& data, int neighbors_num, int threads_num, const std::vector<std::vector<int>>& neighbors_) {

        neighbors_num = std::min(neighbors_num, static_cast<int>(data.demands.size()));

        // Copy info from parsed data.
        vehicle_capacity = data.vehicle_capacity;
        xcoords = std::move(data.xcoords);
        ycoords = std::move(data.ycoords);
        demands = std::move(data.demands);

#ifdef VERBOSE
        Timer timer;
        int progress = 0;
#endif

        if (!neighbors_.empty()) {
            assert(static_cast<int>(neighbors_.size()) == get_vertices_num());
#ifndef NDEBUG
            for (int i = get_vertices_begin(); i < get_vertices_end(); ++i) {
                assert(!neighbors_[i].empty());
            }
#endif
            neighbors = neighbors_;

        } else {
            // Identify the neighbors of each vertex by using a K-d tree, see again the paper cited above, or the explicit cost matrix.
            neighbors.resize(get_vertices_num());

            // (Lambda function)
            // Make sure the first vertex is `i`. Since we are not using all neighbors, if several vertices overlap and the number of
            // neighbors is not large enough we might not have `i` in the neighbors set. Let's cross the fingers and hope it does not
            // happen.
            auto set_first_neighbor = [&](const int i) {
                if (neighbors[i][0] != i) {
                    auto n = 1;
                    while (n < get_vertices_num()) {
                        if (neighbors[i][n] == i) {
                            break;
                        }
                        n++;
                    }
                    std::swap(neighbors[i][0], neighbors[i][n]);
                }
            };

            KDTree kd_tree(xcoords, ycoords);

#pragma omp parallel for schedule(static) num_threads(threads_num)
            for (int i = get_vertices_begin(); i < get_vertices_end(); ++i) {

                neighbors[i] = kd_tree.GetNearestNeighbors(xcoords[i], ycoords[i], neighbors_num);

                set_first_neighbor(i);

                assert(neighbors[i][0] == i);

#ifdef VERBOSE
    #pragma omp critical
                {

                    progress++;
                    if (timer.elapsed_time<std::chrono::seconds>() > 10) {
                        std::cout << "Progress: " << 100 * (progress + 1) / get_vertices_num() << "%\n";
                        timer.reset();
                    }
                }
#endif
            }
        }
    }

}  // namespace cobra