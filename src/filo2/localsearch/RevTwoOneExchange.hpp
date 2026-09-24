#ifndef _FILO2_REVTWOONEEXCHANGE_HPP_
#define _FILO2_REVTWOONEEXCHANGE_HPP_

#include "AbstractOperator.hpp"

namespace cobra {

    template <bool record>
    class RevTwoOneExchange : public AbstractOperator {
    public:
        RevTwoOneExchange(const Instance &instance_, MoveGenerators &moves_, double tolerance_)
            : AbstractOperator(instance_, moves_, tolerance_) { }

        static constexpr bool is_symmetric = false;

    protected:
        inline void pre_processing(__attribute__((unused)) Solution &solution) override { }

        inline double compute_cost(const Solution &solution, const MoveGenerator &move) const override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            const auto iSequenceRem = -solution.get_cost_prev_vertex(iRoute, iPrev) - solution.get_cost_prev_vertex(iRoute, iNext);
            const auto jPrevRem = -solution.get_cost_prev_vertex(jRoute, jNext) - solution.get_cost_prev_vertex(jRoute, jNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNext, iPrev) + this->instance.get_cost(i, j);
            const auto jNextAdd = +this->instance.get_cost(iPrevPrev, jNext) + this->instance.get_cost(jNext, iNext);

            const auto delta = iSequenceAdd + jNextAdd + iSequenceRem + jPrevRem;

            return delta;
        }

        bool is_feasible(Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);


            return (iRoute != jRoute && iPrev != this->instance.get_depot() && jNext != this->instance.get_depot() &&
                    solution.get_route_load(jRoute) - this->instance.get_demand(jNext) + this->instance.get_demand(iPrev) +
                            this->instance.get_demand(i) <=
                        this->instance.get_vehicle_capacity() &&
                    solution.get_route_load(iRoute) + this->instance.get_demand(jNext) - this->instance.get_demand(iPrev) -
                            this->instance.get_demand(i) <=
                        this->instance.get_vehicle_capacity()) ||
                   (iRoute == jRoute && j != iPrev && j != iPrevPrev && jNext != iPrevPrev);
        }

        inline void execute(Solution &solution, const MoveGenerator &move, SparseIntSet &affected_vertices) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);


            affected_vertices.insert(iPrevPrevPrev);
            affected_vertices.insert(iPrevPrev);
            affected_vertices.insert(iPrev);
            affected_vertices.insert(i);
            affected_vertices.insert(iNext);
            affected_vertices.insert(iNextNext);
            affected_vertices.insert(jPrev);
            affected_vertices.insert(j);
            affected_vertices.insert(jNext);
            affected_vertices.insert(jNextNext);
            affected_vertices.insert(jNextNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);

            if (iRoute != jRoute) {
                // Because the depot is shared among all routes, whenever we move a customer i to another route, we should also update SMDs
                // (depot, i) to update the delta of intra-route moves. In particular, the prev/next nodes of the depot are changed now that
                // i is served by a different route.
                // Note: moves (depot, i) and (i, depot) always identify intra-route moves (now with i in a different route!).
                affected_vertices.insert(instance.get_depot());
                this->update_bits.at(instance.get_depot(), UPDATE_BITS_FIRST, true);
            }

            solution.remove_vertex<record>(jRoute, jNext);
            solution.insert_vertex_before<record>(iRoute, iNext, jNext);

            solution.remove_vertex<record>(iRoute, i);
            solution.remove_vertex<record>(iRoute, iPrev);

            solution.insert_vertex_before<record>(jRoute, jNextNext, i);
            solution.insert_vertex_before<record>(jRoute, jNextNext, iPrev);
        }

        void post_processing(__attribute__((unused)) Solution &solution) override { }

        struct Cache12 {
            int v, prev, prevprev, next, nextnext;
            double seqrem, prevrem;
        };

        inline Cache12 prepare_cache12(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache12();
            c.v = vertex;
            const auto route = solution.get_route_index(c.v);
            c.prev = solution.get_prev_vertex(c.v);
            c.prevprev = solution.get_prev_vertex(route, c.prev);
            c.next = solution.get_next_vertex(c.v);
            c.nextnext = solution.get_next_vertex(route, c.next);

            const auto c_v_next = solution.get_cost_prev_vertex(route, c.next);
            c.seqrem = -solution.get_cost_prev_vertex(route, c.prev) - c_v_next;
            c.prevrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }

        inline Cache12 prepare_cache12(const Solution &solution, int vertex, int backup) {

            auto c = Cache12();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route);
            c.prevprev = solution.get_prev_vertex(c.prev);
            c.next = solution.get_first_customer(route);
            c.nextnext = solution.get_next_vertex(c.next);

            const auto c_v_next = solution.get_cost_prev_customer(c.next);
            c.seqrem = -solution.get_cost_prev_customer(c.prev) - c_v_next;
            c.prevrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }

        inline std::pair<double, double> compute_cost_pair(const MoveGenerator &move, const struct Cache12 i, const struct Cache12 j) {

            const auto c_iv_jv = this->moves.get_edge_cost(move);
            const auto c_inext_jnext = this->instance.get_cost(i.next, j.next);

            const auto iSequenceAdd = +this->instance.get_cost(j.nextnext, i.prev) + c_iv_jv;
            const auto jSequenceAdd = +this->instance.get_cost(i.nextnext, j.prev) + c_iv_jv;

            const auto jNextAdd = +this->instance.get_cost(i.prevprev, j.next) + c_inext_jnext;
            const auto iNextAdd = +this->instance.get_cost(j.prevprev, i.next) + c_inext_jnext;


            const auto delta1 = iSequenceAdd + jNextAdd + i.seqrem + j.prevrem;
            const auto delta2 = jSequenceAdd + iNextAdd + j.seqrem + i.prevrem;

            return {delta1, delta2};
        }

        struct Cache1 {
            int v, prev, prevprev, next;
            double seqrem;
        };

        inline Cache1 prepare_cache1(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(c.v);
            c.prev = solution.get_prev_vertex(c.v);
            c.prevprev = solution.get_prev_vertex(route, c.prev);
            c.next = solution.get_next_vertex(c.v);

            const auto c_v_next = solution.get_cost_prev_vertex(route, c.next);
            c.seqrem = -solution.get_cost_prev_vertex(route, c.prev) - c_v_next;

            return c;
        }

        inline Cache1 prepare_cache1(const Solution &solution, int vertex, int backup) {
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route);
            c.prevprev = solution.get_prev_vertex(c.prev);
            c.next = solution.get_first_customer(route);

            const auto c_v_next = solution.get_cost_prev_customer(c.next);
            c.seqrem = -solution.get_cost_prev_customer(c.prev) - c_v_next;

            return c;
        }

        struct Cache2 {
            int v, next, nextnext;
            double prevrem;
        };

        inline Cache2 prepare_cache2(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(c.v);
            c.next = solution.get_next_vertex(c.v);
            c.nextnext = solution.get_next_vertex(route, c.next);

            const auto c_v_next = solution.get_cost_prev_vertex(route, c.next);
            c.prevrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }

        inline Cache2 prepare_cache2(const Solution &solution, int vertex, int backup) {
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.next = solution.get_first_customer(route);
            c.nextnext = solution.get_next_vertex(c.next);

            const auto c_v_next = solution.get_cost_prev_customer(c.next);
            c.prevrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }

        template <typename C1, typename C2>
        inline double compute_cost(const MoveGenerator &move, const C1 i, const C2 j) {

            const auto iSequenceAdd = +this->instance.get_cost(j.nextnext, i.prev) + this->moves.get_edge_cost(move);
            const auto jNextAdd = +this->instance.get_cost(i.prevprev, j.next) + this->instance.get_cost(j.next, i.next);

            const auto delta = iSequenceAdd + jNextAdd + i.seqrem + j.prevrem;

            return delta;
        }
    };

}  // namespace cobra

#endif