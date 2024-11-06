#include "Kruskal.h"
#include <algorithm>
#include "Helpers.h"

uint64_t KruskalMST::Find(uint64_t i)
{
    if (i == parents[i])
    {
        return i;
    }
    uint64_t closerParent = Find(parents[i]);

    // Path compression, reduce the path i -> node -> node -> representative to i -> representative since only the representative matters in this data structure
    parents[i] = closerParent;
    return closerParent;
}

void KruskalMST::MakeUnion(uint64_t i, uint64_t j)
{
    uint64_t rooti = Find(i);
    uint64_t rootj = Find(j);

    // Union by size to reduce the tree size and speed up the find process
    if (rooti == rootj)
    {
        return;
    }

    // Add smallest tree to biggest
    uint64_t sizerooti = sizes[rooti];
    uint64_t sizerootj = sizes[rootj];

    if (sizerooti < sizerootj)
    {
        parents[rooti] = rootj;
    }
    else
    {
        parents[rootj] = rooti;
    }
}

KruskalMST::~KruskalMST()
{
    delete[] parents;
    delete[] sizes;
}

void KruskalMST::FindMST(const std::vector<float2>& vertices, std::vector<Edge>& edges)
{
    // Sort edges by length
    std::sort(edges.begin(), edges.end(), [](Edge& a, Edge& b) { return a.length < b.length; });

    // Initiallize Kruskal algorithm variables
    parents = new uint64_t[vertices.size()];
    sizes = new uint64_t[vertices.size()];
    for (uint64_t i = 0; i < vertices.size(); ++i)
    {
        parents[i] = i;
        sizes[i] = 1;
    }

    // Kruskal algorithm
    for (Edge& edge : edges)
    {
        // If vertices don't belong to the same tree, edge is in MST and we unite their trees together to avoid cycles in the future
        if (Find(edge.start.ID) != Find(edge.end.ID))
        {
            biggestEdge = edge; // Keep track of biggest edge instead of recomputing it later
            MakeUnion(edge.start.ID, edge.end.ID);
        }
    }
}

const Edge KruskalMST::GetBiggestEdge() { return biggestEdge; }
