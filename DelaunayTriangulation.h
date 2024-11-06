#ifndef DELAUNAY_TRIANGULATION_H
#define DELAUNAY_TRIANGULATION_H

class QuadEdge;
struct Edge;

/*
 class implementing the Guibas and Stolfi's divide and conquer algorithm to compute the delaunay triangulation of a set of point
*/
class DelaunayTriangulation
{
public:

    /*
     * @brief Run the whole triangulation process with data initialization before triangulation and trash filtering after
     * @param points The set of points to triangulate
     * @return The list of edges of the triangulation
     */
    void TriangulatePoints(std::vector<float2>& points, std::vector<Edge>& edgesResults);

    ~DelaunayTriangulation();

    //              a.x a.y 1
    // Computes det b.x b.y 1 > 0
    //              c.x c.y 1
    // Returns true if abc forms a counterclockwise triangle
    static bool CCW(float2 a, float2 b, float2 c);

private:

    /**
     * @brief Prepares data for triangulation: sort points by x coordinates (in case of same coordinates decider is on y coordinate), and delete duplicates
     * @param points The list of points to initialize
    */
    void InitData(std::vector<float2>& points);

    /**
     * @brief Recursively finds the Delaunay triangulation for the input set of points. Store said Triangulation in the edges attribute.
     * @param orderedPoints Sorted array of points to triangulate
     * @return leftmost and rightmost edges of the current triangulation, once merge is complete, returns left most and right most edges of the convex hull
    */
    std::pair<QuadEdge*, QuadEdge*> Triangulate(std::vector<float2>& orderedPoints, uint64_t start, uint64_t end, bool vertical, bool dwyer=true);

    // Checks whether d is in the circumcircle defined by the triangle abc
    // Does so by computing:
    //          a.x  a.y  a.x²+a.y² 1 
    //      det b.x  b.y  b.x²+b.y² 1  > 0
    //          c.x  c.y  c.x²+c.y² 1 
    //          d.x  d.y  d.x²+d.y² 1 
    bool InCircle(float2 a, float2 b, float2 c, float2 d);

    // List of Quadedges forming the triangulation
    std::vector<QuadEdge*> edges;
};



/*
 Data structure defined in Guibas and Stolfi's paper.
 It can be used to store various information regarding the surroundings of the edges, but in the case of the Delaunay triangulation, we only need o_next, o_prev and sym.
 The procedures contained in the structure are the ones defined in the paper
*/
class QuadEdge
{

public:

    QuadEdge(float2 org, float2 dest);

    // Returns true if p lies on the left of the edge
    bool LeftOf(float2 p);

    // Returns true if p lies on the right of the edge
    bool RightOf(float2 p);

    /*
     * @brief Creates an edge linking points org and dest, adding it to the list of edges
     * @param CurrentGraph The list of edges defining the Delaunay triangulation
     */
    static QuadEdge* MakeEdge(std::vector<QuadEdge*>& CurrentGraph, float2 org, float2 dest);

    // Connects points a and b by creating an edge between them
    static QuadEdge* Connect(std::vector<QuadEdge*>& CurrentGraph, QuadEdge* a, QuadEdge* b);

    static void DeleteEdge(QuadEdge* e);

    static void Splice(QuadEdge* a, QuadEdge* b);

    double Length();

    float2 m_org;
    float2 m_dest;
    QuadEdge* m_onext;
    QuadEdge* m_oprev;
    QuadEdge* m_sym;
    bool m_data; // Used to discard deleted edges once the triangulation is complete, avoid having to seek the edge to remove in our array every call of delete edge
};

#endif // DELAUNAY_TRIANGULATION_H