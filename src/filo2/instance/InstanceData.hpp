#ifndef _FILO2_INSTANCEDATA_HPP_
#define _FILO2_INSTANCEDATA_HPP_

#include <vector>

#include "filo2/base/NonCopyable.hpp"

namespace cobra {

// All data defining a CVRP instance.
// For coordinates and demand, the depot is assumed to be in the 0-th position.
struct InstanceData : private NonCopyable<InstanceData> {
    // Maximum vehicle capacity.
    int vehicle_capacity;
    // Vertices x coordinates.
    std::vector<double> xcoords;
    // Vertices y coordinates.
    std::vector<double> ycoords;
    // Vertices demands.
    std::vector<int> demands;
};

}  // namespace cobra

#endif