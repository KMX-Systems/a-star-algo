// a-star-algo
// Copyright (c) 2023 KMX Systems. All rights reserved.
#pragma once
#ifndef PCH
    #include <utility>
#endif

namespace kmx::astar
{
    /// Base node class of the nodes used by the algorithm. This class ensure the
    /// existente of 3 values for each node: total score (f), general score (g) and
    /// heuristic score (h). For more details see
    /// http://en.wikipedia.org/wiki/A*_search_algorithm#Algorithm_description. The
    /// descendant of this class has to provide two methods: the distance/cost to
    /// another node (signature Value distance_to( const node_type& node) const) and
    /// the estimated (heuristic) distance/cost to target node (signature void
    /// set_heuristic_score( const node_type& target_node)).
    template <typename _Score>
    class base_node
    {
    public:
        using score_type = _Score;

        base_node() = default;
        base_node(const base_node& node) = default;

        /// Gets the total score (f = g + h).
        score_type total_score() const noexcept { return general_score_ + heuristic_score_; }

        /// Gets the general score (g).
        score_type general_score() const noexcept { return general_score_; }

        /// Sets the general score (g).
        void set_general_score(const score_type value) noexcept { general_score_ = value; }

        /// Gets the heuristic score (h).
        score_type heuristic_score() const noexcept { return heuristic_score_; }

        /// Sets the heuristic score (h).
        void set_heuristic_score(const score_type value) noexcept { heuristic_score_ = value; }

        void clear() noexcept
        {
            general_score_ = {};
            heuristic_score_ = {};
        }

        /// Operator > - used on sorting of the priority queue of open set.
        bool operator>(const base_node& node) const noexcept { return total_score() > node.total_score(); }

    protected:
        score_type general_score_ {};
        score_type heuristic_score_ {};
    };

    /// Dummy beam search functor.
    struct no_beam_search
    {
        bool operator()(auto&, auto&, auto&, auto&) const noexcept { return false; }
    };

    /// @brief Generic C++ implementation of A* algorithm
    /// (http://en.wikipedia.org/wiki/A*_search_algorithm). Features: Fully
    /// customizable internal data structures, step-by-step execution and beam
    /// search support.
    template <typename _Node, typename _PriorityQueue, typename _NeighborEnumerator, typename _Set, typename _SolutionVerifier,
              typename _SolutionMap, typename _BeamSearch = no_beam_search>
    class algo
    {
    public:
        using node_type = _Node;
        using priority_queue_type = _PriorityQueue;
        using set_type = _Set;
        using neighbor_enumerator_type = _NeighborEnumerator;
        using solution_verifier_type = _SolutionVerifier;
        using solution_map_type = _SolutionMap;
        using beam_search_type = _BeamSearch;

        /// @param[in] start_node Start node
        /// @param[in] target_node Target node
        /// @param[in] solution_verifier solution_map_type verifier functor - checks if
        /// the current node is the target node.
        /// @param[in] neighbor_enumerator Enumerator of adjacent nodes
        /// @param[in] beam_search Beach search - filter of the items from open set.
        /// See more details at
        /// http://theory.stanford.edu/~amitp/GameProgramming/Variations.html#S1. By
        /// default the beam search is disable using a dummy filter no_beam_search.
        algo(node_type start_node, node_type target_node, solution_verifier_type solution_verifier,
             neighbor_enumerator_type neighbor_enumerator, beam_search_type beam_search):
            solution_verifier_(std::move(solution_verifier)),
            beam_search_(std::move(beam_search)),
            neighbor_enumerator_(std::move(neighbor_enumerator)),
            target_node_(std::move(target_node))
        {
            start_node.set_heuristic_score(0, target_node_);
            open_set_.insert(start_node);
            priority_open_set_.push(std::move(start_node));
        }

        /// Checks if the solution was found. If the return is true, the solution can
        /// be used by calling @ref solution method.
        bool has_solution() const noexcept { return has_solution_; }

        /// Gets the solution instance.
        const solution_map_type& solution() const noexcept { return solution_; }

        /// Gets the solution instance.
        solution_map_type& solution() noexcept { return solution_; }

        /// Gets the solution verifier.
        const solution_verifier_type& solution_verifier() const noexcept { return solution_verifier_; }

        /// Gets the last used node.
        const node_type& node() const noexcept { return node_; }

        /// Gets the beam search object.
        const beam_search_type& beam_search() const noexcept { return beam_search_; }

        /// @brief algo progress method - useful for fined grained execution, early
        /// exit (see
        /// http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S16)
        /// and algorithm interruption (see
        /// http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S17).
        /// @return Returns true if the algorith should continue. Otherwise the
        /// algorithm cannot continue and
        /// @ref has_solution method has to be checked.
        /// @remark If the method returns false and also @ref has_solution returns
        /// false then no solution was found.
        /// @note The implementation is based on pseudo code from
        /// http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudo_code.
        bool operator()()
        {
            bool can_continue = false;
            if (!open_set_.empty())
            {
                node_ = priority_open_set_.top();
                has_solution_ = solution_verifier_(node_);
                if (!has_solution_)
                {
                    can_continue = true;
                    evaluate_neighbors();
                }
            }

            return can_continue;
        }

    protected:
        void evaluate_neighbors()
        {
            priority_open_set_.pop();
            open_set_.erase(node_);
            closed_set_.insert(node_);
            for (neighbor_enumerator_(node_); neighbor_enumerator_; ++neighbor_enumerator_)
                if (closed_set_.find(*neighbor_enumerator_) == closed_set_.end())
                {
                    node_type& neighbor = *neighbor_enumerator_;
                    const auto tentative_general_score = node_.general_score() + node_.distance_to(neighbor);
                    const bool need_test = open_set_.find(neighbor) == open_set_.end();
                    if (need_test || (tentative_general_score < neighbor.general_score()))
                    {
                        neighbor.set_general_score(tentative_general_score);
                        neighbor.set_heuristic_score(tentative_general_score, target_node_);
                        if (!beam_search_(neighbor, solution_, open_set_, priority_open_set_))
                        {
                            solution_[neighbor] = node_;
                            open_set_.insert(neighbor);
                            priority_open_set_.push(neighbor);
                        }
                    }
                }
        }

        solution_verifier_type solution_verifier_;
        beam_search_type beam_search_;
        neighbor_enumerator_type neighbor_enumerator_;
        priority_queue_type priority_open_set_;
        set_type open_set_;
        set_type closed_set_;
        solution_map_type solution_;
        node_type node_;
        node_type target_node_;
        bool has_solution_ {};
    };
} // namespace kmx::astar
