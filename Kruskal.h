#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <utility>
#include <map>
#include "Helpers.h"

struct Edge;
/*
    class implementing Kruskal's algorithm with the union find data structure/algorithm
*/
class KruskalMST
{
public:
    KruskalMST() = default;
    ~KruskalMST();

    /*
     * @brief Finds MST given the set of vertices and the edges of the graph, no prerequisites on either of the input
     * @param vertices List of vertices, without duplicates
     * @param edges List of edges of the graph
    */
    void FindMST(const std::vector<REC1547::float2>& vertices, std::vector<Edge>& edges);

    // Returns biggest edge of the MST
    const Edge GetBiggestEdge();

    std::vector<Edge> MSTEdges;
private:

    // Find procedure of the union find algorithm
    uint64_t Find(uint64_t i);

    // Union procedure of the union find algorithm
    void MakeUnion(uint64_t i, uint64_t j);

    Edge biggestEdge;

    // Set of parents, defined as a map to track which points has which parent (map can be used since duplicate points were deleted during Delaunay triangulation algorithm)
    uint64_t* parents = nullptr;

    // Size of each vertex, defining the size of the tree they represent if any
    uint64_t* sizes = nullptr;
};

#endif // KRUSKAL_H