#include "AdjacencyList.h"
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/util/Console.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include <boost/graph/incremental_components.hpp>  // needed?

#include <iostream>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              boost::property <boost::vertex_index_t, int,
                              boost::property <boost::vertex_rank_t, int,  // disjoint sets
                              boost::property <boost::vertex_predecessor_t, int> > >,
                              boost::property <boost::edge_weight_t, double> > Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor  Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor    Edge;
typedef boost::graph_traits<Graph>::vertex_iterator    VIterator;
typedef boost::graph_traits<Graph>::edge_iterator      EIterator;
typedef boost::graph_traits<Graph>::adjacency_iterator AdjIterator;

typedef boost::disjoint_sets< boost::property_map<Graph, boost::vertex_rank_t>::type,
                              boost::property_map<Graph, boost::vertex_predecessor_t>::type > DisjointSets;

#define graph_ reinterpret_cast<Graph*>(graphRaw_)
#define disjointSets_ reinterpret_cast<DisjointSets*>(disjointSetsRaw_)

ompl::AdjacencyList::AdjacencyList()
{
    graphRaw_ = new Graph();
    disjointSetsRaw_ = new DisjointSets(boost::get(boost::vertex_rank, *graph_),
                                        boost::get(boost::vertex_predecessor, *graph_));
}

ompl::AdjacencyList::AdjacencyList(int n)
{
    graphRaw_ = new Graph(n);
    disjointSetsRaw_ = new DisjointSets(boost::get(boost::vertex_rank, *graph_),
                                        boost::get(boost::vertex_predecessor, *graph_));

    std::pair<VIterator, VIterator> vRange = boost::vertices(*graph_);
    for(VIterator it = vRange.first; it != vRange.second; ++it)
        disjointSets_->make_set(*it);
}

ompl::AdjacencyList::~AdjacencyList()
{
    delete graph_;
    delete disjointSets_;
    graphRaw_ = NULL;
    disjointSetsRaw_ = NULL;
}

void ompl::AdjacencyList::clear()
{
    boost::mutex::scoped_lock lock(lock_);
    graph_->clear();
}

int ompl::AdjacencyList::addVertex()
{
    boost::mutex::scoped_lock lock(lock_);
    Vertex v = boost::add_vertex(*graph_);
    disjointSets_->make_set(v);

    boost::property_map<Graph, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index, *graph_);
    return vertexIndexMap[v];
}

int ompl::AdjacencyList::numVertices() const
{
    return boost::num_vertices(*graph_);
}

bool ompl::AdjacencyList::vertexExists(int v) const
{
    return v >= 0 && v < numVertices();
}

bool ompl::AdjacencyList::inSameComponent(int v1, int v2) const
{
    if (!vertexExists(v1) || !vertexExists(v2))
        return false;

    return boost::same_component(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *disjointSets_);
}

int ompl::AdjacencyList::numConnectedComponents() const
{
    std::pair<VIterator, VIterator> vRange = boost::vertices(*graph_);
    return disjointSets_->count_sets(vRange.first, vRange.second);
}

int ompl::AdjacencyList::getComponentID(int vtx) const
{
    return disjointSets_->find_set(vtx);
}

bool ompl::AdjacencyList::addEdge(int v1, int v2, double weight)
{
    boost::mutex::scoped_lock lock(lock_);

    // If either of the vertices do not exist, don't add an edge
    if(v1 < 0 || v1 >= numVertices() || v2 < 0 || v2 >= numVertices())
        return false;

    if (edgeExists(v1, v2))
        return false;

    // No self-transitions
    if (v1 == v2)
        return false;

    if (weight < 0)
    {
        std::cout << "weight = " << weight << std::endl;
        throw std::runtime_error("addEdge: Edge weight must be >= 0");
    }

    Edge e;
    bool added = false;
    Vertex vt1 = boost::vertex(v1, *graph_);
    Vertex vt2 = boost::vertex(v2, *graph_);
    tie(e, added) = boost::add_edge(vt1, vt2, *graph_);

    assert(added); // this should never fail

    // Set edge weight
    boost::property_map<Graph, boost::edge_weight_t>::type weights = get(boost::edge_weight, *graph_);
    weights[e] = weight;

    disjointSets_->union_set(vt1, vt2);

    return added;
}

bool ompl::AdjacencyList::removeEdge(int v1, int v2)
{
    boost::mutex::scoped_lock lock(lock_);

    Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
        boost::remove_edge(e, *graph_);
    return exists;
}

int ompl::AdjacencyList::numEdges() const
{
    return boost::num_edges(*graph_);
}

double ompl::AdjacencyList::getEdgeWeight(int v1, int v2) const
{
    Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph, boost::edge_weight_t>::type weights = get(boost::edge_weight, *graph_);
        return weights[e];
    }

    throw std::runtime_error("Edge does not exist");
}

bool ompl::AdjacencyList::setEdgeWeight(int v1, int v2, double weight)
{
    boost::mutex::scoped_lock lock(lock_);

    if (weight < 0)
    {
        std::cout << "Weight: " << weight << std::endl;
        throw std::runtime_error("setEdgeWeight: Edge weight must be >= 0");
    }

    Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph, boost::edge_weight_t>::type weights = get(boost::edge_weight, *graph_);
        weights[e] = weight;
        return true;
    }

    return false;
}

bool ompl::AdjacencyList::edgeExists(int v1, int v2) const
{
    Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    return exists;
}

int ompl::AdjacencyList::numNeighbors(int vtx) const
{
    return boost::degree(boost::vertex(vtx, *graph_), *graph_);
}

void ompl::AdjacencyList::getNeighbors(int vtx, std::vector<int>& nbrs) const
{
    nbrs.resize(numNeighbors(vtx));

    std::pair<AdjIterator, AdjIterator> iterators = boost::adjacent_vertices(boost::vertex(vtx, *graph_), *graph_);
    boost::property_map<Graph, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    int count = 0;
    for (AdjIterator iter = iterators.first; iter != iterators.second; ++iter, ++count)
        nbrs[count] = vertices[*iter];
}

void ompl::AdjacencyList::getNeighbors(int vtx, std::vector<std::pair<int, double> >& nbrs) const
{
    nbrs.resize(numNeighbors(vtx));

    std::pair<AdjIterator, AdjIterator> iterators = boost::adjacent_vertices(boost::vertex(vtx, *graph_), *graph_);
    boost::property_map<Graph, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    boost::property_map<Graph, boost::edge_weight_t>::type weights = get(boost::edge_weight, *graph_);

    int count = 0;
    for (AdjIterator iter = iterators.first; iter != iterators.second; ++iter, ++count)
    {
        Edge e;
        bool exists;
        boost::tie(e, exists) = boost::edge(boost::vertex(vtx, *graph_), *iter, *graph_);
        assert(exists);
        nbrs[count] = std::make_pair(vertices[*iter], weights[e]);
    }
}

bool ompl::AdjacencyList::dijkstra(int v1, int v2, std::vector<int>& path) const
{
    std::vector<double> distances;
    std::vector<int> predecessors;
    dijkstra(v1, predecessors, distances);

    // no solution when predecessor of (non-start) vertex is itself
    if (v2 != v1 && predecessors[v2] == v2)
        return false;

    boost::property_map<Graph, boost::vertex_index_t>::type indexMap = get(boost::vertex_index, *graph_);

    path.clear();
    // Extracting solution path
    Vertex v = boost::vertex(v2, *graph_);
    Vertex start = boost::vertex(v1, *graph_);
    while (v != start)
    {
        path.insert(path.begin(), indexMap[v]);
        v = predecessors[v];
    }
    path.insert(path.begin(), indexMap[v]);
    return true;
}

void ompl::AdjacencyList::dijkstra(int vtx, std::vector<int>& predecessors, std::vector<double>& distances) const
{
    // Locking the graph, since modifications to the graph during search are not supported
    // This call is also the reason why lock_ must be mutable
    boost::mutex::scoped_lock lock(lock_);

    distances.resize(numVertices());
    predecessors.resize(numVertices());

    boost::dijkstra_shortest_paths(*graph_, boost::vertex(vtx, *graph_),
                                   boost::predecessor_map(&predecessors[0]).
                                   distance_map(&distances[0]));
}
