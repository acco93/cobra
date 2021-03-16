/**
 * Instance data structure.
 */

#ifndef COBRA_ABSTRACTINSTANCE_HPP
#define COBRA_ABSTRACTINSTANCE_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>
#include "NonCopyable.hpp"
#include "AbstractInstanceParser.hpp"
#include "mm.hpp"
#include "Flat2DVector.hpp"

namespace cobra {

class Instance : private NonCopyable<Instance> {

 public:

    template<template<bool> class Parser, bool round_costs>
    static std::optional<Instance> make(const std::string &path) {

        static_assert(std::is_base_of<AbstractInstanceParser<round_costs>, Parser<round_costs>>::value, "Parser not derived from AbstractInstanceParser");

        auto parser = Parser<round_costs>(path);
        auto ok = parser.parse();

        if (ok) { return Instance(parser); }
        return std::nullopt;

    }

    // Move constructor
    Instance(Instance &&other) noexcept {
        matrix_size = other.matrix_size;
        capacity = other.capacity;
        x_coordinates = std::move(other.x_coordinates);
        y_coordinates = std::move(other.y_coordinates);
        demands = std::move(other.demands);
        depot = other.depot;
        customers_num = other.customers_num;
        customers_begin = other.customers_begin;
        customers_end = other.customers_end;
        costs_matrix = std::move(other.costs_matrix);
        neighbors = std::move(other.neighbors);
    }

    ~Instance() = default;

    /**
     * Returns the depot node index.
     * @return depot
     */
    inline int get_depot() const { return 0; };

    /**
     * Returns the vehicle capacity.
     * @return vehicle capacity
     */
    inline int get_vehicle_capacity() const { return capacity; };

    /**
     * Returns the number of customers.
     * @return number of customers
     */
    inline int get_customers_num() const { return customers_num; };

    /**
     * Returns the index of the first customer.
     * @return first customer index
     */
    inline int get_customers_begin() const { return customers_begin; };

    /**
     * Returns the index after the last customer.
     * It is commonly used as follows
     * for (auto i = customers_begin(); i < customers_end(); i++) { ... }
     * @return index after the last customer
     */
    inline int get_customers_end() const { return customers_end; };

    /**
     * Returns the total number of vertices (depot + customers)
     * @return total number of vertices
     */
    inline int get_vertices_num() const { return matrix_size; };

    /**
     * Returns the index of the first vertex.
     * @return first vertex index
     */
    inline int get_vertices_begin() const { return 0; };

    /**
     * Returns the index after the last vertex.
     * It is commonly used as follows
     * for (auto i = vertices_begin(); i < vertices_end(); i++) { ... }
     * @return index after the last vertex
     */
    inline int get_vertices_end() const { return matrix_size; };

    /**
     * Returns the cost of the edge between i and j.
     * @param i vertex
     * @param j vertex
     * @return cost of the (i, j) edge
     */
    inline float get_cost(int i, int j) const {

        return costs_matrix.at(i, j);

    };

    /**
     * Returns the vertex demand.
     * @param i vertex
     * @return vertex demand
     */
    inline int get_demand(int i) const { return demands[i]; };

    /**
     * Returns the x-coordinate of a vertex.
     * @param i vertex
     * @return vertex x-coordinate
     */
    inline float get_x_coordinate(int i) const { return x_coordinates[i]; };

    /**
     * Returns the y-coordinate of a vertex.
     * @param i vertex
     * @return vertex y-coordinate
     */
    inline float get_y_coordinate(int i) const { return y_coordinates[i]; };

    /**
     * Returns a reference to an array of vertices sorted according to increasing cost from i. The array includes i
     * itself in the first position. Note that this must be enforced by the implementer, it is not always automatically
     * obtained just by sorting the vertices (e.g. when i and some other vertices overlap).
     * @param i vertex
     * @return reference to the neighbors array
     */
    inline const std::vector<int> &get_neighbors_of(int i) const { return neighbors[i]; };

 private:

    int matrix_size = 0;
    int capacity = 0;
    std::vector<float> x_coordinates;
    std::vector<float> y_coordinates;
    std::vector<int> demands;
    int depot = 0;
    int customers_num = 0;
    int customers_begin = 0;
    int customers_end = 0;
    Flat2DVector<float> costs_matrix;
    std::vector<std::vector<int>> neighbors;

    template<bool round_costs>
    explicit Instance(AbstractInstanceParser<round_costs> &parser) {

        // Move the properties from parser to instance

        capacity = parser.get_vehicle_capacity();
        x_coordinates = std::move(parser.get_x_coordinates());
        y_coordinates = std::move(parser.get_y_coordinates());
        demands = std::move(parser.get_demands());
        matrix_size = demands.size();

        depot = 0;
        customers_num = matrix_size - 1;
        customers_begin = 1;
        customers_end = matrix_size;
        costs_matrix = std::move(parser.get_costs_matrix());
        neighbors = std::move(parser.get_neighbors());

    }

};

}

#endif //COBRA_ABSTRACTINSTANCE_HPP
