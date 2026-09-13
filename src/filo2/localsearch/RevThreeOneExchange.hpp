#ifndef _FILO2_REVTHREEONEEXCHANGE_HPP_
#define _FILO2_REVTHREEONEEXCHANGE_HPP_

#include "AbstractOperator.hpp"

namespace cobra {

    template <bool record>
    class RevThreeOneExchange : public AbstractOperator {
    public:
        RevThreeOneExchange(const Instance &instance_, MoveGenerators &moves_, double tolerance_)
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
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            const auto iSequenceRem = -solution.get_cost_prev_vertex(iRoute, iPrevPrev) - solution.get_cost_prev_vertex(iRoute, iNext);
            const auto jSequenceRem = -solution.get_cost_prev_vertex(jRoute, jNext) - solution.get_cost_prev_vertex(jRoute, jNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNext, iPrevPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNext) + this->instance.get_cost(jNext, iNext);

            return iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;
        }

        bool is_feasible(Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                    jNext != this->instance.get_depot() &&
                    solution.get_route_load(jRoute) - this->instance.get_demand(jNext) + this->instance.get_demand(i) +
                            this->instance.get_demand(iPrev) + this->instance.get_demand(iPrevPrev) <=
                        this->instance.get_vehicle_capacity() &&
                    solution.get_route_load(iRoute) + this->instance.get_demand(jNext) - this->instance.get_demand(i) -
                            this->instance.get_demand(iPrev) - this->instance.get_demand(iPrevPrev) <=
                        this->instance.get_vehicle_capacity()) ||
                   (iRoute == jRoute && j != iPrev && j != iPrevPrev && jNext != iPrevPrev &&
                    jNext != solution.get_prev_vertex(iRoute, iPrevPrev));
        }

        inline void execute(Solution &solution, const MoveGenerator &move, SparseIntSet &affected_vertices) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);
            const auto iPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);

            affected_vertices.insert(iPrevPrevPrevPrev);
            affected_vertices.insert(iPrevPrevPrev);
            affected_vertices.insert(iPrevPrev);
            affected_vertices.insert(iPrev);
            affected_vertices.insert(i);
            affected_vertices.insert(iNext);
            affected_vertices.insert(iNextNext);
            affected_vertices.insert(iNextNextNext);
            affected_vertices.insert(jPrev);
            affected_vertices.insert(j);
            affected_vertices.insert(jNext);
            affected_vertices.insert(jNextNext);
            affected_vertices.insert(jNextNextNext);
            affected_vertices.insert(jNextNextNextNext);

            this->update_bits.at(iPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNext, UPDATE_BITS_FIRST, true);
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

            solution.remove_vertex<record>(iRoute, i);
            solution.remove_vertex<record>(iRoute, iPrev);
            solution.remove_vertex<record>(iRoute, iPrevPrev);

            solution.insert_vertex_before<record>(jRoute, jNextNext, i);
            solution.insert_vertex_before<record>(jRoute, jNextNext, iPrev);
            solution.insert_vertex_before<record>(jRoute, jNextNext, iPrevPrev);

            solution.remove_vertex<record>(jRoute, jNext);

            solution.insert_vertex_before<record>(iRoute, iNext, jNext);
        }

        void post_processing(__attribute__((unused)) Solution &solution) override { }

        struct Cache12 {
            int v, next, prevprev, prevprevprev, nextnext;
            double sequrem, nextrem;
        };

        inline Cache12 prepare_cache12(const Solution &solution, int vertex) {

            assert(vertex != this->instance.get_depot());
            auto c = Cache12();
            c.v = vertex;
            const auto prev = solution.get_prev_vertex(c.v);
            c.next = solution.get_next_vertex(c.v);
            const auto route = solution.get_route_index(c.v);
            c.prevprev = solution.get_prev_vertex(route, prev);
            c.prevprevprev = solution.get_prev_vertex(route, c.prevprev);
            c.nextnext = solution.get_next_vertex(route, c.next);

            const auto c_v_next = solution.get_cost_prev_vertex(route, c.next);

            // c.v = i in (i, j)
            c.sequrem = -solution.get_cost_prev_vertex(route, c.prevprev) - c_v_next;
            // c.v = j in (i, j)
            c.nextrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }


        inline Cache12 prepare_cache12(const Solution &solution, int vertex, int backup) {


            auto c = Cache12();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            const auto prev = solution.get_last_customer(route);
            c.next = solution.get_first_customer(route);
            c.prevprev = solution.get_prev_vertex(prev);
            c.prevprevprev = solution.get_prev_vertex(route, c.prevprev);
            c.nextnext = solution.get_next_vertex(c.next);

            const auto c_v_next = solution.get_cost_prev_customer(c.next);

            // c.v = i in (i, j)
            c.sequrem = -solution.get_cost_prev_vertex(route, c.prevprev) - c_v_next;
            // c.v = j in (i, j)
            c.nextrem = -c_v_next - solution.get_cost_prev_vertex(route, c.nextnext);

            return c;
        }

        inline std::pair<double, double> compute_cost_pair(const MoveGenerator &move, const struct Cache12 i, const struct Cache12 j) {

            const auto c_iv_jv = this->moves.get_edge_cost(move);
            const auto c_inext_jnext = this->instance.get_cost(j.next, i.next);

            const auto delta1 = this->instance.get_cost(j.nextnext, i.prevprev) + c_iv_jv +
                                this->instance.get_cost(i.prevprevprev, j.next) + c_inext_jnext + i.sequrem + j.nextrem;
            const auto delta2 = this->instance.get_cost(i.nextnext, j.prevprev) + c_iv_jv +
                                this->instance.get_cost(j.prevprevprev, i.next) + c_inext_jnext + j.sequrem + i.nextrem;

            return {delta1, delta2};
        }

        struct Cache1 {
            int v, next, prevprev, prevprevprev;
            double sequrem;
        };

        inline Cache1 prepare_cache1(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            const auto prev = solution.get_prev_vertex(c.v);
            c.next = solution.get_next_vertex(c.v);
            const auto route = solution.get_route_index(c.v);
            c.prevprev = solution.get_prev_vertex(route, prev);
            c.prevprevprev = solution.get_prev_vertex(route, c.prevprev);
            c.sequrem = -solution.get_cost_prev_vertex(route, c.prevprev) - solution.get_cost_prev_vertex(route, c.next);
            return c;
        }

        inline Cache1 prepare_cache1(const Solution &solution, int vertex, int backup) {
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            const auto prev = solution.get_last_customer(route);
            c.next = solution.get_first_customer(route);
            c.prevprev = solution.get_prev_vertex(prev);
            c.prevprevprev = solution.get_prev_vertex(route, c.prevprev);
            c.sequrem = -solution.get_cost_prev_vertex(route, c.prevprev) - solution.get_cost_prev_customer(c.next);
            return c;
        }

        struct Cache2 {
            int v, next, nextnext;
            double nextrem;
        };

        inline Cache2 prepare_cache2(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            c.next = solution.get_next_vertex(c.v);
            const auto route = solution.get_route_index(c.v);
            c.nextnext = solution.get_next_vertex(route, c.next);
            c.nextrem = -solution.get_cost_prev_vertex(route, c.next) - solution.get_cost_prev_vertex(route, c.nextnext);
            return c;
        }

        inline Cache2 prepare_cache2(const Solution &solution, int vertex, int backup) {
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.next = solution.get_first_customer(route);
            c.nextnext = solution.get_next_vertex(c.next);
            c.nextrem = -solution.get_cost_prev_customer(c.next) - solution.get_cost_prev_vertex(route, c.nextnext);
            return c;
        }

        template <typename C1, typename C2>
        inline double compute_cost(const MoveGenerator &move, const C1 i, const C2 j) {
            const auto c_iv_jv = this->moves.get_edge_cost(move);
            const auto c_inext_jnext = this->instance.get_cost(j.next, i.next);

            return this->instance.get_cost(j.nextnext, i.prevprev) + c_iv_jv + this->instance.get_cost(i.prevprevprev, j.next) +
                   c_inext_jnext + i.sequrem + j.nextrem;
        }
    };

}  // namespace cobra

#endif