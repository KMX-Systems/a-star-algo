// a-star-algo
// Copyright (c) 2023 KMX Systems. All rights reserved.
//#include "stdafx.h"
#include "astar_algo.hpp"
#include <iostream>
#include <queue>
#include <set>
#include <vector>

using namespace std;

namespace kmx::astar::demo
{
    class xy_node: public base_node<int>
    {
    public:
        using value_type = int;
        using base_type = base_node<int>;
        using neighbors_type = std::vector<int>;

        xy_node(const int id = 0, const value_type x = 0, const value_type y = 0): _id(id), _x(x), _y(y) {}
        xy_node(const xy_node& node) = default;

        operator int() const noexcept { return id(); }

        int id() const noexcept { return _id; }
        void set_id(const int value) { _id = value; }

        value_type x() const noexcept { return _x; }
        void set_x(const value_type value) { _x = value; }

        value_type y() const noexcept { return _y; }
        void set_y(const value_type value) { _y = value; }

        value_type distance_to(const xy_node& node) const noexcept { return _x * node._x + _y + node._y; }

        void set_heuristic_score(const xy_node& targetNode) noexcept { base_type::set_heuristic_score(distance_to(targetNode)); }

        neighbors_type neighbors;

    protected:
        value_type _id, _x, _y;
    };

    struct solution_verifier
    {
        const int _node_id;
        solution_verifier(const int node_id) noexcept: _node_id(node_id) {}

        bool operator()(const xy_node& n) const noexcept { return n.id() == _node_id; }
    };

    using xy_node_list = std::vector<xy_node>;

    class enumerator
    {
    public:
        enumerator(xy_node_list& node_list): _node_list(node_list) {}

        operator bool() const noexcept { return _node != nullptr; }

        void operator()(const xy_node& node)
        {
            _neighbors = node.neighbors.begin();
            _neighbors_end = node.neighbors.end();
            if (_neighbors == _neighbors_end)
            {
                _node = nullptr;
            }
            else
            {
                _node = &_node_list[*_neighbors];
            }
        }

        void operator++()
        {
            if (_node)
            {
                if (_neighbors + 1 != _neighbors_end)
                {
                    ++_neighbors;
                    _node = &_node_list[*_neighbors];
                }
                else
                {
                    _node = nullptr;
                }
            }
        }

        xy_node& operator*() noexcept { return *_node; }
        const xy_node& operator*() const noexcept { return *_node; }

        xy_node* operator->() noexcept { return _node; }
        const xy_node* operator->() const noexcept { return _node; }

    private:
        xy_node::neighbors_type::const_iterator _neighbors {};
        xy_node::neighbors_type::const_iterator _neighbors_end {};
        xy_node* _node {};
        xy_node_list& _node_list;
    };

    void test1(xy_node_list& node_list)
    {
        node_list.clear();

        xy_node n;

        n.set_id(0);
        n.set_x(0);
        n.set_y(5);
        n.neighbors.clear();
        n.neighbors.push_back(1);
        n.neighbors.push_back(2);
        node_list.push_back(n);

        n.set_id(1);
        n.set_x(3);
        n.set_y(6);
        n.neighbors.clear();
        n.neighbors.push_back(3);
        node_list.push_back(n);

        n.set_id(2);
        n.set_x(4);
        n.set_y(3);
        n.neighbors.clear();
        n.neighbors.push_back(4);
        n.neighbors.push_back(5);
        node_list.push_back(n);

        n.set_id(3);
        n.set_x(6);
        n.set_y(9);
        n.neighbors.clear();
        n.neighbors.push_back(6);
        n.neighbors.push_back(7);
        node_list.push_back(n);

        n.set_id(4);
        n.set_x(7);
        n.set_y(3);
        n.neighbors.clear();
        n.neighbors.push_back(8);
        n.neighbors.push_back(10);
        node_list.push_back(n);

        n.set_id(5);
        n.set_x(6);
        n.set_y(1);
        n.neighbors.clear();
        n.neighbors.push_back(8);
        node_list.push_back(n);

        n.set_id(6);
        n.set_x(8);
        n.set_y(6);
        n.neighbors.clear();
        n.neighbors.push_back(7);
        n.neighbors.push_back(10);
        node_list.push_back(n);

        n.set_id(7);
        n.set_x(11);
        n.set_y(8);
        n.neighbors.clear();
        n.neighbors.push_back(9);
        node_list.push_back(n);

        n.set_id(8);
        n.set_x(10);
        n.set_y(2);
        n.neighbors.clear();
        n.neighbors.push_back(11);
        node_list.push_back(n);

        n.set_id(9);
        n.set_x(13);
        n.set_y(6);
        n.neighbors.clear();
        node_list.push_back(n);

        n.set_id(10);
        n.set_x(8);
        n.set_y(6);
        n.neighbors.clear();
        n.neighbors.push_back(12);
        node_list.push_back(n);

        n.set_id(11);
        n.set_x(13);
        n.set_y(0);
        n.neighbors.clear();
        node_list.push_back(n);

        n.set_id(12);
        n.set_x(17);
        n.set_y(3);
        n.neighbors.clear();
        node_list.push_back(n);
    }

    struct solution_item
    {
        const int from;
        const int to;

        solution_item(const int from, const int to = 0): from(from), to(to) {}

        bool operator<(const solution_item& item) const noexcept { return from < item.from; }
        bool operator==(const solution_item& item) const noexcept = default;
    };

    using solution = set<solution_item>;
    using algo = astar::algo<xy_node, priority_queue<xy_node>, enumerator, set<int>, solution_verifier, solution>;

    void print(const solution& s, const xy_node& target_node)
    {
        int id = target_node;
        while (1)
        {
            auto item = s.find(id);
            if (item != s.end())
            {
                cout << id << ' ';
                id = item->to;
            }
            else
                break;
        }

        cout << '\n';
    }
}

int main()
{
    using namespace kmx::astar::demo;

    xy_node_list nodeList;
    test1(nodeList);

    const xy_node& startNode = nodeList[0];
    const xy_node& targetNode = nodeList[nodeList.size() - 1];

    algo as_algo(startNode, targetNode, solution_verifier(targetNode), enumerator(nodeList), {});

    unsigned steps = 0;
    while (as_algo())
    {
        ++steps;
    }

    if (as_algo.has_solution())
    {
        cout << "steps=" << steps << " path: ";
        print(as_algo.solution(), targetNode);
    }

    return 0;
}
