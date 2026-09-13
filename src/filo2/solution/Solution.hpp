#ifndef _FILO2_SOLUTION_HPP_
#define _FILO2_SOLUTION_HPP_

#include <fstream>
#include <iostream>
#include <vector>

#include "../base/FixedSizeValueStack.hpp"
#include "../base/LRUCache.hpp"
#include "../base/macro.hpp"
#include "../instance/Instance.hpp"

namespace cobra {

    // Class representing a CVRP solution.
    // A few high-level notes:
    // - Route is not a first class concept (e.g., there is no Route class, but all operations on routes go through this Solution class).
    // - Routes are stored as doubly linked lists (implemented with a prev and next vectors).
    // - There is a single depot, and this makes it quite a special node since it belongs to all routes, but it cannot be used to identify
    // any specific route. For this reason, there are methods such as `get_route_index` that either take a single customer in input or two
    // vertices: the vertex for which we are interested in getting the route index, and a fallback that is a vertex (different from the
    // previous one), that should be used in case the former is the depot.
    class Solution {

    public:
        // Dummy value used to identify an invalid customer.
        static inline const int dummy_vertex = -1;

        // Dummy value used to identify an invalid route.
        static inline const int dummy_route = 0;

        explicit Solution(const Instance &instance_) : Solution(instance_, instance_.get_vertices_num()) { }

        Solution(const Instance &instance_, int history_len)
            : instance(instance_),
              solution_cost(INFINITY),
              max_number_routes(instance_.get_customers_num() + 1),
              routes_pool(max_number_routes,
                          [](int index) {
                              return index + 1;
                          }),
              depot_node({Solution::dummy_route, 0}),
              routes_list(max_number_routes),
              customers_list(instance_.get_vertices_num()),
              cache(history_len, instance_.get_vertices_num()) { }

        Solution(const Solution &source)
            : instance(source.instance),
              solution_cost(INFINITY),
              max_number_routes(instance.get_customers_num() + 1),
              routes_pool(max_number_routes,
                          [](int index) {
                              return index + 1;
                          }),
              depot_node({Solution::dummy_route, 0}),
              routes_list(max_number_routes),
              customers_list(instance.get_vertices_num()),
              cache(source.cache) {
            copy(source);
        }

        ~Solution() = default;

        Solution &operator=(const Solution &source) {
            if (this == &source) {
                return *this;
            }
            copy(source);
            return *this;
        }

        bool operator==(const Solution &other) const {
            bool diff = false;
            if (std::fabs(solution_cost - other.solution_cost) >= 0.01) {
                std::cout << "Different solution costs: (this)" << solution_cost << " != (other)" << other.solution_cost << "\n";
                diff = true;
            }
            for (int i = instance.get_customers_begin(); i < instance.get_customers_end(); ++i) {
                if (get_prev_vertex(i) != other.get_prev_vertex(i) || get_next_vertex(i) != other.get_next_vertex(i)) {
                    std::cout << "Different prev/next for vertex " << i << ":\n";
                    std::cout << "This:\n";
                    print(get_route_index(i));
                    std::cout << "Other:\n";
                    other.print(other.get_route_index(i));
                    diff = true;
                }
            }
            return !diff;
        }

        bool operator!=(const Solution &other) const {
            return !(*this == other);
        }

        // Reset a solution. Currently a newly constructed solution object needs to be reset before usage. That's ugly and must be fixed.
        // Why is the solution cost set to infinity in the constructor? Weird.
        void reset() {

            solution_cost = 0.0;

            routes_pool.reset();

            depot_node.first_route = Solution::dummy_route;
            depot_node.num_routes = 0;

            for (int r = 0; r < max_number_routes; r++) {
                reset_route(r);
            }

            for (int i = 0; i < instance.get_vertices_num(); i++) {
                reset_vertex(i);
            }

            cache.clear();

            undo_list1.clear();
            do_list1.clear();
            do_list2.clear();
        }

        // Returns the solution cost.
        inline double get_cost() const {
            return solution_cost;
        }

        // Returns the number of routes in the solution.
        inline int get_routes_num() const {
            return depot_node.num_routes;
        }

        // Builds a one-customer route and returns the route index.
        template <bool record>
        int build_one_customer_route(const int customer) {

            assert(!is_customer_in_solution(customer));
            assert(customer != instance.get_depot());

            const auto route = request_route();

            if constexpr (record) {
                do_list1.emplace_back(Action::build_one_customer_route(customer));
                undo_list1.emplace_back(Action::remove_one_customer_route(customer));

#ifndef NDEBUG
                Solution copy = *this;
                copy.apply<false>(do_list1.back());
                copy.apply<false>(undo_list1.back());
                assert(copy == *this);
#endif
            }

            customers_list[customer].prev = instance.get_depot();
            customers_list[customer].next = instance.get_depot();
            customers_list[customer].route_ptr = route;
            customers_list[customer].c_prev_curr = instance.get_cost(instance.get_depot(), customer);

            // Head insert the route in the list.
            const auto next_route = depot_node.first_route;
            routes_list[route].next = next_route;
            depot_node.first_route = route;
            routes_list[route].prev = Solution::dummy_route;
            routes_list[next_route].prev = route;

            routes_list[route].first_customer = customer;
            routes_list[route].last_customer = customer;
            routes_list[route].load = instance.get_demand(customer);
            routes_list[route].size = 1;
            routes_list[route].c_prev_curr = customers_list[customer].c_prev_curr;

            solution_cost += 2.0 * customers_list[customer].c_prev_curr;

            cache.insert(customer);

            routes_list[route].needs_cumulative_load_update = true;

            return route;
        }

        // Builds a route with the given customers.
        template <bool record>
        int build_route(const std::vector<int> &customers) {
            assert(!customers.empty());

            const int route = build_one_customer_route<record>(customers[0]);
            for (int i = 1; i < static_cast<int>(customers.size()); ++i) {
                insert_vertex_before<record>(route, instance.get_depot(), customers[i]);
            }

            return route;
        }

        // Returns the index of the route serving the given customer. Returns Solution::dummy_route if the customer is not served.
        inline int get_route_index(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].route_ptr;
        }

        // Returns the index of the route serving the given vertex. If the vertex is the depot, the fallback is used to identify the route
        // index.
        inline int get_route_index(const int vertex, const int fallback) const {
            if (unlikely(vertex == instance.get_depot())) {
                return customers_list[fallback].route_ptr;
            } else {
                return customers_list[vertex].route_ptr;
            }
        }

        // Returns the load of the given route.
        inline int get_route_load(const int route) const {
            return routes_list[route].load;
        }

        // Returns the index of the first route in the solution.
        inline int get_first_route() const {
            return depot_node.first_route;
        }

        // Returns the index of the next route in the list of routes.
        inline int get_next_route(const int route) const {
            return routes_list[route].next;
        }

        // Returns the index after the last route. Useful to check for termination when looping through routes.
        inline int get_end_route() const {
            return dummy_route;
        }

        // Returns whether the route does not serve any customer. Note that empty routes must always be removed from the solution.
        inline bool is_route_empty(const int route) const {
            return routes_list[route].load == 0;
        }

        // Removes the vertex from the given route.
        template <bool record>
        double remove_vertex(const int route, const int vertex) {

            assert(contains_vertex(route, vertex));

            if constexpr (record) {

                const int next = get_next_vertex(route, vertex);
                // We should not call remove_vertex on an empty route.
                assert(!is_route_empty(route));
                assert(vertex != instance.get_depot() || next != instance.get_depot());
                assert(next != vertex);

                // Removing the depot does not change the route size.
                if (get_route_size(route) == 1 && vertex != instance.get_depot()) {
                    do_list1.emplace_back(Action::remove_one_customer_route(vertex));
                    undo_list1.emplace_back(Action::build_one_customer_route(vertex));
                } else {
                    int fallback = get_next_vertex(route, next);
                    // There are some weird edge cases where fallback can be equal to vertex. As an example consider the route: 18 -> 183 ->
                    // 18, which is missing the depot. If vertex=18, then next=183 and fallback=18. In this case, we want 183 to be the
                    // fallback.
                    if (vertex == fallback) {
                        assert((is_missing_depot(route) && get_route_size(route) == 2) ||
                               (vertex == instance.get_depot() && get_route_size(route) == 1));
                        fallback = next;
                    }
                    assert(fallback != vertex);
                    // If vertex is depot, we use next to detect the route.
                    do_list1.emplace_back(Action::remove_vertex(vertex, /*fallback=*/next));
                    undo_list1.emplace_back(Action::insert_vertex_before(next, vertex, fallback));
                }

#ifndef NDEBUG
                Solution copy = *this;
                copy.apply<false>(do_list1.back());
                copy.apply<false>(undo_list1.back());
                assert(copy == *this);
#endif
            }

            if (unlikely(vertex == instance.get_depot())) {

                assert(!is_route_empty(route));

                const auto next = routes_list[route].first_customer;
                const auto prev = routes_list[route].last_customer;

                cache.insert(vertex);
                cache.insert(prev);
                cache.insert(next);

                set_prev_vertex_ptr(route, next, prev);
                set_next_vertex_ptr(route, prev, next);

                routes_list[route].first_customer = Solution::dummy_vertex;
                routes_list[route].last_customer = Solution::dummy_vertex;

                assert(next != instance.get_depot());
                customers_list[next].c_prev_curr = instance.get_cost(prev, next);

                const auto delta = +customers_list[next].c_prev_curr - instance.get_cost(prev, vertex) - instance.get_cost(vertex, next);

                solution_cost += delta;

                routes_list[route].needs_cumulative_load_update = true;

                return delta;

            } else {

                assert(contains_vertex(route, vertex));
                assert(instance.get_depot() != vertex);

                const auto next = customers_list[vertex].next;
                const auto prev = customers_list[vertex].prev;

                cache.insert(vertex);
                cache.insert(prev);
                cache.insert(next);

                if (vertex == routes_list[route].first_customer) {
                    routes_list[route].first_customer = next;
                    set_prev_vertex_ptr(route, next, instance.get_depot());  // Next might be the root of the route.
                } else if (vertex == routes_list[route].last_customer) {
                    routes_list[route].last_customer = prev;
                    set_next_vertex_ptr(route, prev, instance.get_depot());  // Prev might be the root of the route.
                } else {
                    customers_list[prev].next = next;  // If vertex != route.first_customer, then prev is not the root.
                    customers_list[next].prev = prev;  // If vertex != route.last_customer, then next is not the root.
                }

                routes_list[route].load -= instance.get_demand(vertex);
                routes_list[route].size -= 1;

                const auto c_prev_next = instance.get_cost(prev, next);

                if (next == instance.get_depot()) {
                    routes_list[route].c_prev_curr = c_prev_next;
                } else {
                    customers_list[next].c_prev_curr = c_prev_next;
                }

                const auto delta = +c_prev_next - instance.get_cost(prev, vertex) - instance.get_cost(vertex, next);

                solution_cost += delta;

                // TODO: is this really necessary?
                reset_vertex(vertex);

                routes_list[route].needs_cumulative_load_update = true;

                return delta;
            }
        }

        // Removes all customers from the given route, and removes the route from the solution.
        template <bool record>
        void empty_and_remove_route(const int route) {
            assert(!is_route_empty(route));
            int curr = get_first_customer(route);
            while (curr != instance.get_depot()) {
                const int next = get_next_vertex(curr);
                remove_vertex<record>(route, curr);
                curr = next;
            }
            remove_route(route);
        }

        // Removes the route from the solution. The route must be empty.
        void remove_route(const int route) {
            assert(is_route_empty(route));
            release_route(route);
        }

        // Returns the first customer of the route.
        inline int get_first_customer(const int route) const {
            return routes_list[route].first_customer;
        }

        // Returns the last customer of the route.
        inline int get_last_customer(const int route) const {
            return routes_list[route].last_customer;
        }

        // Returns the customer after the given one in its route.
        inline int get_next_vertex(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].next;
        }

        // Returns the vertex after the given one in its route. Vertex must belong to route. This is the right method if vertex might be the
        // depot.
        inline int get_next_vertex(const int route, const int vertex) const {
            assert(contains_vertex(route, vertex));
            if (unlikely(vertex == instance.get_depot())) {
                return routes_list[route].first_customer;
            } else {
                return customers_list[vertex].next;
            }
        }

        // Returns the customer before the given one in its route.
        inline int get_prev_vertex(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].prev;
        }

        // Returns the vertex before the given one in its route. Vertex must belong to route. This is the right method if vertex might be
        // the depot.
        inline int get_prev_vertex(const int route, const int vertex) const {
            assert(contains_vertex(route, vertex));
            if (unlikely(vertex == instance.get_depot())) {
                return get_last_customer(route);
            } else {
                return get_prev_vertex(vertex);
            }
        }

        // Inserts `vertex` before `where` in `route`. `where` must belong to `route` and `vertex` must be unserved.
        template <bool record>
        void insert_vertex_before(const int route, const int where, const int vertex) {

            if constexpr (record) {

                // Tails and split may work on empty routes...
                if (is_route_empty(route)) {
                    assert(where == instance.get_depot());
                    assert(vertex != instance.get_depot());
                    do_list1.emplace_back(Action::build_one_customer_route(vertex));
                    undo_list1.emplace_back(Action::remove_one_customer_route(vertex));
                } else {
                    const int fallback = get_next_vertex(route, where);

                    assert(where != instance.get_depot() || fallback != instance.get_depot());
                    assert(vertex != fallback);
                    do_list1.emplace_back(Action::insert_vertex_before(where, vertex, fallback));
                    undo_list1.emplace_back(Action::remove_vertex(vertex, fallback));
                }

#ifndef NDEBUG
                Solution copy = *this;
                copy.apply<false>(do_list1.back());
                copy.apply<false>(undo_list1.back());
                assert(copy == *this);
#endif
            }

            assert(where != vertex);

            if (unlikely(vertex == instance.get_depot())) {

                assert(routes_list[route].first_customer == Solution::dummy_vertex);
                assert(routes_list[route].last_customer == Solution::dummy_vertex);
                assert(where != instance.get_depot());

                assert(!is_route_empty(route));

                const auto prev = customers_list[where].prev;

                cache.insert(prev);
                cache.insert(where);

                assert(prev != instance.get_depot());

                routes_list[route].first_customer = where;
                routes_list[route].last_customer = prev;

                customers_list[prev].next = instance.get_depot();
                customers_list[where].prev = instance.get_depot();

                routes_list[route].c_prev_curr = instance.get_cost(prev, instance.get_depot());

                double old_cost_prev_where = customers_list[where].c_prev_curr;
                customers_list[where].c_prev_curr = instance.get_cost(instance.get_depot(), where);

                const auto delta = +routes_list[route].c_prev_curr + customers_list[where].c_prev_curr - old_cost_prev_where;

                solution_cost += delta;

            } else {

                assert(!is_customer_in_solution(vertex));
                assert(vertex != instance.get_depot());
                assert(where == instance.get_depot() || get_route_index(where) == route);

                const auto prev = get_prev_vertex(route, where);

                cache.insert(prev);
                cache.insert(where);

                customers_list[vertex].next = where;
                customers_list[vertex].prev = prev;
                customers_list[vertex].route_ptr = route;

                set_next_vertex_ptr(route, prev, vertex);
                set_prev_vertex_ptr(route, where, vertex);

                double old_cost_prev_where;
                const auto c_vertex_where = instance.get_cost(vertex, where);
                if (where == instance.get_depot()) {
                    old_cost_prev_where = routes_list[route].c_prev_curr;
                    routes_list[route].c_prev_curr = c_vertex_where;
                } else {
                    old_cost_prev_where = customers_list[where].c_prev_curr;
                    customers_list[where].c_prev_curr = c_vertex_where;
                }
                customers_list[vertex].c_prev_curr = instance.get_cost(prev, vertex);

                const auto delta = +customers_list[vertex].c_prev_curr + c_vertex_where - old_cost_prev_where;

                solution_cost += delta;
                routes_list[route].load += instance.get_demand(vertex);
                routes_list[route].size += 1;
            }

            routes_list[route].needs_cumulative_load_update = true;
        }

        // Reverses the sub-path identified by vertex_begin and vertex_end.
        template <bool record>
        void reverse_route_path(const int route, const int vertex_begin, const int vertex_end) {

            const auto stop = get_next_vertex(route, vertex_end);
            if (stop == vertex_begin) {
                return;
            }

            if constexpr (record) {
                do_list1.emplace_back(Action::reverse_route_path(vertex_begin, vertex_end));
                undo_list1.emplace_back(Action::reverse_route_path(vertex_end, vertex_begin));

#ifndef NDEBUG
                Solution copy = *this;
                copy.apply<false>(do_list1.back());
                copy.apply<false>(undo_list1.back());
                assert(copy == *this);
#endif
            }

            assert(vertex_begin != vertex_end);

            const auto pre = get_prev_vertex(route, vertex_begin);

            const auto c_pre_begin = get_cost_prev_vertex(route, vertex_begin);

            const double c_pre_vertex_end = instance.get_cost(pre, vertex_end);
            const double c_vertex_begin_stop = instance.get_cost(stop, vertex_begin);

            cache.insert(pre);
            cache.insert(stop);

            auto curr = vertex_begin;
            do {

                cache.insert(curr);

                const auto prev = get_prev_vertex(route, curr);
                const auto next = get_next_vertex(route, curr);

                if (curr == instance.get_depot()) {
                    routes_list[route].last_customer = next;
                    routes_list[route].first_customer = prev;
                    assert(next != instance.get_depot());
                    routes_list[route].c_prev_curr = customers_list[next].c_prev_curr;
                } else {
                    customers_list[curr].prev = next;
                    customers_list[curr].next = prev;
                    customers_list[curr].c_prev_curr = get_cost_prev_vertex(route, next);
                }

                curr = next;

            } while (curr != stop);

            if (vertex_end == pre && vertex_begin == stop) {
                // `vertex_begin` and `vertex_end` are contiguous.
                if (vertex_end == instance.get_depot()) {
                    routes_list[route].c_prev_curr = c_pre_begin;
                } else {
                    customers_list[vertex_end].c_prev_curr = c_pre_begin;
                }
            } else {

                set_next_vertex_ptr(route, vertex_begin, stop);
                set_next_vertex_ptr(route, pre, vertex_end);

                if (vertex_end == instance.get_depot()) {
                    routes_list[route].last_customer = pre;
                    routes_list[route].c_prev_curr = c_pre_vertex_end;
                } else {
                    customers_list[vertex_end].prev = pre;
                    customers_list[vertex_end].c_prev_curr = c_pre_vertex_end;
                }

                if (stop == instance.get_depot()) {
                    routes_list[route].last_customer = vertex_begin;
                    routes_list[route].c_prev_curr = c_vertex_begin_stop;
                } else {
                    customers_list[stop].prev = vertex_begin;
                    customers_list[stop].c_prev_curr = c_vertex_begin_stop;
                }
            }

            const auto delta = -instance.get_cost(pre, vertex_begin) - instance.get_cost(vertex_end, stop) + c_pre_vertex_end +
                               c_vertex_begin_stop;

            solution_cost += delta;

            routes_list[route].needs_cumulative_load_update = true;
        }

        // Appends the customer of `route_to_append` to `route` and removes `route_to_append` from the solution.
        int append_route(const int route, const int route_to_append) {

            const int route_end = routes_list[route].last_customer;
            const int route_to_append_start = routes_list[route_to_append].first_customer;

            assert(route_end != instance.get_depot());
            assert(route_to_append_start != instance.get_depot());

            customers_list[route_end].next = route_to_append_start;
            customers_list[route_to_append_start].prev = route_end;
            customers_list[route_to_append_start].c_prev_curr = instance.get_cost(route_end, route_to_append_start);

            routes_list[route].last_customer = routes_list[route_to_append].last_customer;
            routes_list[route].load += routes_list[route_to_append].load;
            routes_list[route].size += routes_list[route_to_append].size;
            routes_list[route].c_prev_curr = routes_list[route_to_append].c_prev_curr;

            const double delta = +customers_list[route_to_append_start].c_prev_curr - instance.get_cost(route_end, instance.get_depot()) -
                                 instance.get_cost(instance.get_depot(), route_to_append_start);

            solution_cost += delta;

            cache.insert(route_end);

            for (int curr = route_to_append_start; curr != instance.get_depot(); curr = customers_list[curr].next) {
                customers_list[curr].route_ptr = route;

                cache.insert(curr);
            }

            release_route(route_to_append);

            routes_list[route].needs_cumulative_load_update = true;

            return route;
        }

        // Generate a string representation of the given route.
        std::string to_string(const int route) const {
            std::string str;
            str += "[" + std::to_string(route) + "] ";
            str += std::to_string(instance.get_depot()) + " ";
            for (int curr = routes_list[route].first_customer; curr != instance.get_depot(); curr = customers_list[curr].next) {
                str += std::to_string(curr) + " ";
            }
            str += std::to_string(instance.get_depot());
            return str;
        }

        // Prints a string representation of the given route.
        void print(const int route) const {
            if (is_missing_depot(route)) {
                std::cout << "Route " << route << " is in an INCONSISTENT state: missing the depot. It cannot be accessed without it.\n";
            } else {
                std::cout << to_string(route) << " (" << get_route_load(route) << ") " << get_route_cost(route) << "\n";
            }
        }

        // Prints the solution.
        void print() const {
            for (auto route = depot_node.first_route; route != Solution::dummy_route; route = routes_list[route].next) {
                print(route);
            }
            std::cout << "Solution cost = " << solution_cost << "\n";
        }

        // Returns the route's cumulative load before the given customer included.
        inline int get_route_load_before_included(const int customer) {
            assert(customer != instance.get_depot());
            const int route = customers_list[customer].route_ptr;
            if (routes_list[route].needs_cumulative_load_update) {
                update_cumulative_route_loads(route);
                routes_list[route].needs_cumulative_load_update = false;
            }
            return customers_list[customer].load_before;
        }

        // Returns the route's cumulative load after the given customer included.
        inline int get_route_load_after_included(const int customer) {
            assert(customer != instance.get_depot());
            const int route = customers_list[customer].route_ptr;
            if (routes_list[route].needs_cumulative_load_update) {
                update_cumulative_route_loads(route);
                routes_list[route].needs_cumulative_load_update = false;
            }
            return customers_list[customer].load_after;
        }

        // Returns whether the given route index is currently served in the solution.
        inline bool is_route_in_solution(const int route) const {
            return routes_list[route].in_solution;
        }

        // Returns whether the given customer is currently served in the solution.
        inline bool is_customer_in_solution(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].route_ptr != Solution::dummy_route;
        }

        // Returns whether the given vertex is currently served in the solution. Use this method, if vertex could be the depot.
        inline bool is_vertex_in_solution(const int vertex) const {
            return vertex == instance.get_depot() || is_customer_in_solution(vertex);
        }

        // Returns whether the given vertex is served by route. Alwats returns true when vertex is the depot.
        inline bool contains_vertex(const int route, const int vertex) const {
            assert(vertex >= instance.get_vertices_begin() && vertex < instance.get_vertices_end() && route >= 0 &&
                   route < max_number_routes);
            return customers_list[vertex].route_ptr == route || vertex == instance.get_depot();
        }

        // Returns the number of customers served by the given route.
        inline int get_route_size(const int route) const {
            return routes_list[route].size;
        }

        // Swaps the customers [iNext, ..., lastCustomer(iRoute)] from `iRoute`  and [j, lastCustomer(jRoute)] from `jRoute`.
        //
        // [iRoute] = depot, o, o, o, i, iNext, o, o, o, depot
        //                              \ /
        //                               X___
        //                              /    |
        // [jRoute] = depot, o, o, o, jPrev, j, o, o, o, depot
        //
        // Definitely not the best picture, but this replaces (i, iNext) with (i, j) and (jPrev, j) with (jPrev, jNext).
        template <bool record>
        void swap_tails(const int i, const int iRoute, const int j, const int jRoute) {

            assert(i != instance.get_depot());
            assert(j != instance.get_depot());
            assert(iRoute != jRoute);
            assert(contains_vertex(iRoute, i));
            assert(contains_vertex(jRoute, j));

            const auto iNext = customers_list[i].next;

            auto curr = j;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex<record>(jRoute, curr);
                insert_vertex_before<record>(iRoute, iNext, curr);
                curr = next;
            }

            curr = iNext;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex<record>(iRoute, curr);
                insert_vertex_before<record>(jRoute, instance.get_depot(), curr);
                curr = next;
            }

            routes_list[iRoute].needs_cumulative_load_update = true;
            routes_list[jRoute].needs_cumulative_load_update = true;
        }

        // Replaces (i, iNext) with (i, j) and reverts (depot, j). Replaces (j, jNext) with (iNext, jNext) and reverts (iNext, depot).
        //
        // [iRoute] = depot, o, o, o, i  iNext<-o<-o<-o<-depot
        //                            |     |
        //                            |     |
        // [jRoute] = depot<-o<-o<-o<-j  jNext, o, o, o, depot
        template <bool record>
        void split(const int i, const int iRoute, const int j, const int jRoute) {

            assert(i != instance.get_depot());
            assert(j != instance.get_depot());

            const auto iNext = customers_list[i].next;
            const auto jNext = customers_list[j].next;

            auto curr = j;
            while (curr != instance.get_depot()) {
                const auto prev = customers_list[curr].prev;
                remove_vertex<record>(jRoute, curr);
                insert_vertex_before<record>(iRoute, iNext, curr);
                curr = prev;
            }

            auto before = jNext;
            curr = iNext;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex<record>(iRoute, curr);
                insert_vertex_before<record>(jRoute, before, curr);
                before = curr;
                curr = next;
            }

            routes_list[iRoute].needs_cumulative_load_update = true;
            routes_list[jRoute].needs_cumulative_load_update = true;
        }

        // Returns the cost of the arc (prev, vertex) where prev is the predecessor of vertex in route. Vertex must be served by route. Use
        // this method if vertex could be the depot.
        inline double get_cost_prev_vertex(int route, int vertex) const {
            if (vertex == instance.get_depot()) {
                return routes_list[route].c_prev_curr;
            } else {
                return customers_list[vertex].c_prev_curr;
            }
        }

        // Returns the cost of the arc (prev, customer) where prev is the predecessor of customer in route. Customer must be served by
        // route.
        inline double get_cost_prev_customer(int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].c_prev_curr;
        }

        // Returns the cost of the arc (prev, depot) where prev is the predecessor of depot in route.
        inline double get_cost_prev_depot(int route) const {
            return routes_list[route].c_prev_curr;
        }

        // Returns the route cost. This is currently a linear procedure in the instance size. Use with caution!
        double get_route_cost(const int route) const {
            int curr = routes_list[route].first_customer;
            double sum = instance.get_cost(instance.get_depot(), curr);
            while (curr != instance.get_depot()) {
                const int next = customers_list[curr].next;
                sum += instance.get_cost(curr, next);
                curr = next;
            }

            return sum;
        }

        // Clears the set of recently modified vertices.
        inline void clear_svc() {
            cache.clear();
        }

        // Returns the cache containing recently modified vertices. Note that it is not safe to perform any operations on the solution while
        // iterating the returned set, so store the elements somewhere else if necessary.
        inline const LRUCache &get_svc() const {
            return cache;
        }

        // Returns the first element in the set of recently modified vertices.
        inline int get_svc_begin() const {
            return cache.begin();
        }

        // Returns the element after the given one in the set of recently modified vertices.
        inline int get_svc_next(const int i) const {
            return cache.get_next(i);
        }

        // Returns an invalid vertex, useful to terminate iteration over the cache.
        inline int get_svc_end() const {
            return cache.end();
        }

        // Returns the number of elements in the set of recently modified vertices.
        inline int get_svc_size() const {
            return cache.size();
        }

        // Returns whether the route does not violate load constraints.
        inline bool is_load_feasible(const int route) const {
            return routes_list[route].load <= instance.get_vehicle_capacity();
        }

        // Returns whether the solution does not violate load constraints.
        inline bool is_load_feasible() const {
            for (auto r = get_first_route(); r != dummy_route; r = get_next_route(r)) {
                if (!is_load_feasible(r)) {
                    return false;
                }
            }
            return true;
        }

        // Stores the solution to file.
        static void store_to_file(const cobra::Instance &instance, const cobra::Solution &solution, const std::string &path) {
            auto out_stream = std::ofstream(path);
            for (auto route = solution.get_first_route(), idx = 1; route != cobra::Solution::dummy_route;
                 route = solution.get_next_route(route), idx++) {
                out_stream << "Route #" << idx << ":";
                for (auto customer = solution.get_first_customer(route); customer != instance.get_depot();
                     customer = solution.get_next_vertex(customer)) {
                    out_stream << " " << std::to_string(customer);
                }
                out_stream << "\n";
            }
            out_stream << "Cost " << std::to_string(solution.get_cost());
        }

        // Applies the do-list 1 to solution.
        void apply_do_list1(Solution &solution) const {
            for (int i = 0; i < static_cast<int>(do_list1.size()); ++i) {
                assert(solution.is_applicable(do_list1[i]));
                solution.apply</*record=*/false>(do_list1[i]);
            }
            assert(solution.is_feasible());
        }

        // Applied the do-list 2 to solution.
        void apply_do_list2(Solution &solution) const {
            for (int i = 0; i < static_cast<int>(do_list2.size()); ++i) {
                assert(solution.is_applicable(do_list2[i]));
                solution.apply</*record=*/false>(do_list2[i]);
            }
            assert(solution.is_feasible());
        }

        // Appends the do-list 1 to the do-list 2.
        void append_do_list1_to_do_list2() {
            for (const auto &entry : do_list1) {
                do_list2.emplace_back(entry);
            }
        }

        // Applies the undo-list 1 to solution.
        void apply_undo_list1(Solution &solution) const {
            for (int i = static_cast<int>(undo_list1.size()) - 1; i >= 0; --i) {
                assert(solution.is_applicable(undo_list1[i]));
                solution.apply</*record=*/false>(undo_list1[i]);
            }
            assert(solution.is_feasible());
        }

        void undo_and_pop_from_list1() {
            assert(!undo_list1.empty());
            apply</*record=*/false>(undo_list1.back());
            undo_list1.pop_back();
            do_list1.pop_back();
        }

        // Clears the do-list 1.
        inline void clear_do_list1() {
            do_list1.clear();
        }

        // Clears the do-list 2.
        inline void clear_do_list2() {
            do_list2.clear();
        }

        // Clears the undo-list 1.
        inline void clear_undo_list1() {
            undo_list1.clear();
        }

        inline bool is_missing_depot(const int route) const {
            return get_first_customer(route) == Solution::dummy_vertex;
        }

        // Returns whether the solution is CVRP feasible. This is a very expensive procedure. Must only be used for debugging purposes.
        bool is_feasible(const bool error_on_load_infeasible = true, const bool verbose = false) const;

        enum class ActionType { INSERT_VERTEX, REMOVE_VERTEX, REVERSE_ROUTE_PATH, BUILD_ONE_CUSTOMER_ROUTE, REMOVE_ONE_CUSTOMER_ROUTE };

        class Action {
        public:
            static Action insert_vertex_before(int where, int vertex, int fallback) {
                return Action(ActionType::INSERT_VERTEX, vertex, where, fallback);
            }
            static Action remove_vertex(int vertex, int fallback) {
                return Action(ActionType::REMOVE_VERTEX, vertex, Solution::dummy_vertex, fallback);
            }
            static Action reverse_route_path(int begin, int end) {
                return Action(ActionType::REVERSE_ROUTE_PATH, begin, end, Solution::dummy_vertex);
            }
            static Action build_one_customer_route(int customer) {
                return Action(ActionType::BUILD_ONE_CUSTOMER_ROUTE, customer, Solution::dummy_vertex, Solution::dummy_vertex);
            }
            static Action remove_one_customer_route(int customer) {
                return Action(ActionType::REMOVE_ONE_CUSTOMER_ROUTE, customer, Solution::dummy_vertex, Solution::dummy_vertex);
            }
            ActionType type;
            int i, j;
            // Additional node used to identify a route.
            // When working with delta changes generated by different solvers, we cannot safely use the route index to identify a route in a
            // completely different solution. For this reason we rely on an additional fallback node, which will be used to retrieve the
            // route index during an insertion/removal whenever j is the depot.
            int fallback;

            void print() const {
                std::cout << "Action: ";
                switch (type) {
                case ActionType::INSERT_VERTEX:
                    std::cout << "INSERT_VERTEX";
                    break;
                case ActionType::REMOVE_VERTEX:
                    std::cout << "REMOVE_VERTEX";
                    break;
                case ActionType::REVERSE_ROUTE_PATH:
                    std::cout << "REVERSE_ROUTE_PATH";
                    break;
                case ActionType::BUILD_ONE_CUSTOMER_ROUTE:
                    std::cout << "BUILD_ONE_CUSTOMER_ROUTE";
                    break;
                case ActionType::REMOVE_ONE_CUSTOMER_ROUTE:
                    std::cout << "REMOVE_ONE_CUSTOMER_ROUTE";
                    break;
                }
                std::cout << "\n";
                std::cout << "i=" << i << "\n";
                std::cout << "j=" << j << "\n";
                std::cout << "fallback=" << fallback << "\n";
            }

        private:
            Action(ActionType type, int i, int j, int fallback) : type(type), i(i), j(j), fallback(fallback) { }
        };

        // Returns whether applying action to solution would return a structurally feasible solution. Note that load constraints are not
        // checked as we expect that subsequent actions may fix potential load infeasibilities.
        bool is_applicable(const Action &action) const {
            switch (action.type) {
            case cobra::Solution::ActionType::BUILD_ONE_CUSTOMER_ROUTE:
                if (is_customer_in_solution(action.i)) return false;
                break;
            case cobra::Solution::ActionType::REMOVE_ONE_CUSTOMER_ROUTE:
                if (!is_customer_in_solution(action.i)) return false;
                if (get_route_size(get_route_index(action.i)) != 1) return false;
                break;
            case cobra::Solution::ActionType::INSERT_VERTEX: {
                if (action.i != instance.get_depot() && is_customer_in_solution(action.i)) return false;
                if (!is_vertex_in_solution(action.j)) return false;
                if (!is_vertex_in_solution(action.fallback)) return false;
                const int route = get_route_index(action.j, action.fallback);
                const int route_fallback = get_route_index(action.fallback, action.j);
                if (route != route_fallback) return false;
                if (action.i == instance.get_depot() && !is_missing_depot(route)) return false;
                break;
            }
            case cobra::Solution::ActionType::REMOVE_VERTEX: {
                if (action.i != instance.get_depot() && !is_customer_in_solution(action.i)) return false;
                if (!is_vertex_in_solution(action.fallback)) return false;
                const int route = get_route_index(action.i, action.fallback);
                const int route_fallback = get_route_index(action.fallback, action.i);
                if (route != route_fallback) return false;
                if (action.i == instance.get_depot() && is_missing_depot(route)) return false;
                break;
            }
            case cobra::Solution::ActionType::REVERSE_ROUTE_PATH: {
                if (!is_vertex_in_solution(action.i)) return false;
                if (!is_vertex_in_solution(action.j)) return false;
                const int i_route = get_route_index(action.i, action.j);
                const int j_route = get_route_index(action.j, action.i);
                if (i_route != j_route) return false;
                break;
            }
            }
            return true;
        }

        inline const std::vector<Action> &get_do_list1() const {
            return do_list1;
        }

        inline const std::vector<Action> &get_undo_list1() const {
            return undo_list1;
        }

        inline const std::vector<Action> &get_do_list2() const {
            return do_list2;
        }

        // Applies action to solution and returns the route involved in the action.
        template <bool record>
        int apply(const Action &action) {
            assert(is_applicable(action));
            switch (action.type) {
            case ActionType::BUILD_ONE_CUSTOMER_ROUTE:
                assert(action.i != 0 /*depot*/);
                assert(!is_vertex_in_solution(action.i));
                return build_one_customer_route<record>(action.i);
            case ActionType::REMOVE_ONE_CUSTOMER_ROUTE: {
                assert(action.i != 0 /*depot*/);
                assert(is_vertex_in_solution(action.i));
                const int route = get_route_index(action.i);
                assert(get_route_size(route) == 1);
                remove_vertex<record>(route, action.i);
                remove_route(route);
                return route;
            }
            case ActionType::INSERT_VERTEX: {
                assert(action.i == 0 /*depot is always served*/ || !is_customer_in_solution(action.i));
                assert(is_vertex_in_solution(action.j));
                assert(is_vertex_in_solution(action.fallback));
                if (action.j == 0 && action.fallback == 0) {
                    std::cout << "ActionType::INSERT_VERTEX (one customer route creation)\n";
                    return build_one_customer_route<record>(action.i);
                } else {
                    const int route = get_route_index(action.j, action.fallback);
                    insert_vertex_before<record>(route, action.j, action.i);
                    return route;
                }
            }
            case ActionType::REMOVE_VERTEX: {
                assert(action.i == 0 /*depot is always served*/ || is_customer_in_solution(action.i));
                assert(action.i != 0 || action.fallback != 0);
                assert(is_vertex_in_solution(action.fallback));
                const int route = get_route_index(action.i, action.fallback);
                remove_vertex<record>(route, action.i);
                if (is_route_empty(route)) remove_route(route);
                return route;
            }
            case ActionType::REVERSE_ROUTE_PATH: {
                assert(is_vertex_in_solution(action.i));
                assert(is_vertex_in_solution(action.j));
                assert(get_route_index(action.i, action.j) == get_route_index(action.j, action.i));
                const int route = get_route_index(action.i, action.j);
                reverse_route_path<record>(route, action.i, action.j);
                return route;
            }
            }
            // Not reached.
            assert(true == false);
            return Solution::dummy_route;
        }

    private:
        // Performs a deep copy a the given source solution. It should not really be used too often if the instance is big.
        void copy(const Solution &source) {

#ifndef NDEBUG
                // This is commented out as in debug we are now doing a lot of copies...
                // std::cout << "WARNING: performing a full solution copy could be very expensive. Consider using do/undo lists.\n";
#endif
            routes_pool = source.routes_pool;
            depot_node = source.depot_node;
            customers_list = source.customers_list;
            routes_list = source.routes_list;
            solution_cost = source.solution_cost;
            cache = source.cache;

            do_list1 = source.do_list1;
            undo_list1 = source.undo_list1;
        }

        void reset_route(const int route) {
            routes_list[route].load = 0;
            routes_list[route].size = 0;
            routes_list[route].first_customer = Solution::dummy_vertex;
            routes_list[route].last_customer = Solution::dummy_vertex;
            routes_list[route].prev = Solution::dummy_route;
            routes_list[route].next = Solution::dummy_route;
            routes_list[route].needs_cumulative_load_update = true;
            routes_list[route].in_solution = false;
        }

        void reset_vertex(const int customer) {
            customers_list[customer].next = Solution::dummy_vertex;
            customers_list[customer].prev = Solution::dummy_vertex;
            customers_list[customer].route_ptr = Solution::dummy_route;
        }

        inline void set_next_vertex_ptr(const int route, const int vertex, const int next) {
            if (unlikely(vertex == instance.get_depot())) {
                routes_list[route].first_customer = next;
            } else {
                customers_list[vertex].next = next;
            }
        }

        inline void set_prev_vertex_ptr(const int route, const int vertex, const int prev) {
            if (unlikely(vertex == instance.get_depot())) {
                routes_list[route].last_customer = prev;
            } else {
                customers_list[vertex].prev = prev;
            }
        }

        inline int request_route() {
            assert(!routes_pool.is_empty());

            const auto route = routes_pool.get();
            routes_list[route].in_solution = true;

            depot_node.num_routes++;

            return route;
        }

        inline void release_route(const int route) {

            const auto prevRoute = routes_list[route].prev;
            const auto nextRoute = routes_list[route].next;

            routes_list[prevRoute].next = nextRoute;
            routes_list[nextRoute].prev = prevRoute;
            depot_node.num_routes--;

            // Head remove.
            if (depot_node.first_route == route) {
                depot_node.first_route = nextRoute;
            }

            reset_route(route);

            routes_pool.push(route);
        }

        void update_cumulative_route_loads(const int route) {

            assert(!is_route_empty(route));

            auto prev = routes_list[route].first_customer;

            customers_list[prev].load_before = instance.get_demand(prev);
            customers_list[prev].load_after = routes_list[route].load;

            auto curr = customers_list[prev].next;

            while (curr != instance.get_depot()) {

                customers_list[curr].load_before = customers_list[prev].load_before + instance.get_demand(curr);
                customers_list[curr].load_after = customers_list[prev].load_after - instance.get_demand(prev);

                prev = curr;
                curr = customers_list[curr].next;
            }
        }

        // Structure representing the depot.
        struct DepotNode {
            // Index of the first route in the route linked list.
            int first_route;
            // Current number of routes in the route linked list.
            int num_routes;
        };

        // Structure representing a customer.
        struct CustomerNode {
            // Index of the next customer in the route.
            int next;
            // Index of the previous customer in the route.
            int prev;
            // Index of the route serving this customer.
            int route_ptr;
            // Cumulative load sum from this customer up to the depot in this route.
            int load_after;
            // Cumulative load sum from the depot up to this customer (included) in this route.
            int load_before;
            // Cost of the arc (prev, this customer) where prev is the predecessor of this customer.
            double c_prev_curr;
        };

        // Structure representing a route.
        struct RouteNode {
            // Index of the first customer in the route.
            int first_customer;
            // Index of the last customer in the route.
            int last_customer;
            // Overall load of the route.
            int load;
            // Index of the next route in solution.
            int next;
            // Index of the previous route in solution.
            int prev;
            // Number of customers in the route.
            int size;
            // Whether customers of this route require an update to their `load_after` and `load_before`.
            bool needs_cumulative_load_update;
            // Whether this route is in solution.
            bool in_solution;
            // Cost of the arc (depot, last customer in this route).
            double c_prev_curr;
        };

        const Instance &instance;
        double solution_cost;

        // TODO: Is this really necessary?
        const int max_number_routes;

        FixedSizeValueStack<int> routes_pool;
        struct DepotNode depot_node;
        std::vector<RouteNode> routes_list;
        std::vector<CustomerNode> customers_list;
        cobra::LRUCache cache;

        // This is not very nice.
        std::vector<Action> undo_list1;
        std::vector<Action> do_list1;
        std::vector<Action> do_list2;
    };

}  // namespace cobra


#endif