cmake_minimum_required(VERSION 3.22)
project(filo2x)

set(CMAKE_CXX_STANDARD 20)

set(WARNING_FLAGS "-Wall -Wextra -Wpedantic -Wuninitialized")
set(OPT_FLAGS "-O3 -march=native -flto=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -g -fsanitize=address -fsanitize=undefined -fsanitize-address-use-after-scope") 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_VERBOSE "Enable verbose output" ON)

message("-- Build options")

if(ENABLE_VERBOSE)
    message("--- Verbose output ENABLED")
    add_definitions(-DVERBOSE)
else()
    message("--- Verbose output DISABLED")
endif()


# Use includes starting from the root directory.
include_directories("src")

set(LIBFILO2

    src/filo2/base/BinaryHeap.hpp
    src/filo2/base/BitMatrix.hpp
    src/filo2/base/FixedSizeValueStack.hpp
    src/filo2/base/Flat2DVector.hpp
    src/filo2/base/functor.hpp
    src/filo2/base/KDTree.cpp
    src/filo2/base/KDTree.hpp
    src/filo2/base/LRUCache.hpp
    src/filo2/base/macro.hpp
    src/filo2/base/NonCopyable.hpp
    src/filo2/base/PrettyPrinter.hpp
    src/filo2/base/SmallFlatMap.hpp
    src/filo2/base/SmallFlatSet.hpp
    src/filo2/base/StringUtils.hpp
    src/filo2/base/Timer.hpp
    src/filo2/base/VectorView.hpp
    src/filo2/base/SparseIntSet.hpp
    src/filo2/base/Welford.hpp

    src/filo2/instance/Instance.cpp
    src/filo2/instance/Instance.hpp
    src/filo2/instance/InstanceData.hpp
    src/filo2/instance/Parser.cpp
    src/filo2/instance/Parser.hpp

    src/filo2/localsearch/AbstractOperator.hpp
    src/filo2/localsearch/EjectionChain.hpp
    src/filo2/localsearch/LocalSearch.hpp
    src/filo2/localsearch/OneOneExchange.hpp
    src/filo2/localsearch/OneZeroExchange.hpp
    src/filo2/localsearch/RevThreeOneExchange.hpp
    src/filo2/localsearch/RevThreeThreeExchange.hpp
    src/filo2/localsearch/RevThreeTwoExchange.hpp
    src/filo2/localsearch/RevThreeZeroExchange.hpp
    src/filo2/localsearch/RevTwoOneExchange.hpp
    src/filo2/localsearch/RevTwoTwoExchange.hpp
    src/filo2/localsearch/RevTwoZeroExchange.hpp
    src/filo2/localsearch/SplitExchange.hpp
    src/filo2/localsearch/TailsExchange.hpp
    src/filo2/localsearch/ThreeOneExchange.hpp
    src/filo2/localsearch/ThreeThreeExchange.hpp
    src/filo2/localsearch/ThreeTwoExchange.hpp
    src/filo2/localsearch/ThreeZeroExchange.hpp
    src/filo2/localsearch/TwoOneExchange.hpp
    src/filo2/localsearch/TwoOptExchange.hpp
    src/filo2/localsearch/TwoTwoExchange.hpp
    src/filo2/localsearch/TwoZeroExchange.hpp

    src/filo2/movegen/MoveGenerators.hpp

    src/filo2/opt/bpp.hpp
    src/filo2/opt/RuinAndRecreate.hpp
    src/filo2/opt/SimulatedAnnealing.hpp

    src/filo2/solution/savings.hpp
    src/filo2/solution/Solution.hpp
    src/filo2/solution/Solution.cpp

)

find_package(OpenMP REQUIRED)

add_executable(filo2x 

    ${LIBFILO2}

    src/filo2x/Dispatcher.cpp
    src/filo2x/Dispatcher.hpp
    src/filo2x/main.cpp
    src/filo2x/Message.hpp
    src/filo2x/Parameters.hpp
    src/filo2x/SharedQueue.hpp
    src/filo2x/Solver.cpp
    src/filo2x/Solver.hpp
    src/filo2x/SolverCoreopt.cpp
    src/filo2x/Statistics.hpp
)

target_link_libraries(filo2x PUBLIC OpenMP::OpenMP_CXX)
