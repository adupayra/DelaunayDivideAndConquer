#include "DelaunayTriangulation.h"
#include <algorithm>
#include "Helpers.h"

bool x_first(const float2& p1, const float2& p2)
{
    return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}
bool y_first(const float2& p1, const float2& p2)
{
    return p1.y > p2.y || (p1.y == p2.y && p1.x > p2.x);
}

QuadEdge::QuadEdge(float2 org, float2 dest) : m_org(org), m_dest(dest), m_onext(nullptr), m_oprev(nullptr), m_sym(nullptr), m_data(false)
{
}

inline
bool QuadEdge::LeftOf(float2 p)
{
    return DelaunayTriangulation::CCW(p, m_org, m_dest);
}

inline
bool QuadEdge::RightOf(float2 p)
{
    return DelaunayTriangulation::CCW(p, m_dest, m_org);
}

QuadEdge* QuadEdge::MakeEdge(std::vector<QuadEdge*>& CurrentGraph, float2 org, float2 dest)
{
    QuadEdge* e = new QuadEdge(org, dest);
    QuadEdge* esym = new QuadEdge(dest, org);

    e->m_sym = esym;
    esym->m_sym = e;

    e->m_onext = e;
    e->m_oprev = e;

    esym->m_onext = esym;
    esym->m_oprev = esym;
    CurrentGraph.push_back(e);
    return e;
}

QuadEdge* QuadEdge::Connect(std::vector<QuadEdge*>& CurrentGraph, QuadEdge* a, QuadEdge* b)
{
    QuadEdge* e = MakeEdge(CurrentGraph, a->m_dest, b->m_org);
    Splice(e, a->m_sym->m_oprev);
    Splice(e->m_sym, b);
    return e;
}

void QuadEdge::DeleteEdge(QuadEdge* e)
{
    Splice(e, e->m_oprev);
    Splice(e->m_sym, e->m_sym->m_oprev);

    // Above splice is "logically" deleting the edge by moving pointers around, but they remain in the list of edges of the graph, so we mark them to remove later
    e->m_data = true;
    e->m_sym->m_data = true;
}

void QuadEdge::Splice(QuadEdge* a, QuadEdge* b)
{
    if (a == b)
    {
        return;
    }

    a->m_onext->m_oprev = b;
    b->m_onext->m_oprev = a;

    QuadEdge* temp = a->m_onext;
    a->m_onext = b->m_onext;
    b->m_onext = temp;
}

void DelaunayTriangulation::InitData(std::vector<float2>& points)
{
    std::sort(points.begin(), points.end());
    auto it = std::unique(points.begin(), points.end());
    points.erase(it, points.end());
}

inline
bool DelaunayTriangulation::InCircle(float2 a, float2 b, float2 c, float2 d)
{
    float a1 = a.x - d.x;
    float a2 = a.y - d.y;
    float b1 = b.x - d.x;
    float b2 = b.y - d.y;
    float c1 = c.x - d.x;
    float c2 = c.y - d.y;
    float a3 = a1 * a1 + a2 * a2;
    float b3 = b1 * b1 + b2 * b2;
    float c3 = c1 * c1 + c2 * c2;
    float det = a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - (a3 * b2 * c1 + a1 * b3 * c2 + a2 * b1 * c3);
    return det > 0;
}

inline
bool DelaunayTriangulation::CCW(float2 a, float2 b, float2 c)
{
    return (((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))) > 0;
}

std::pair<QuadEdge*, QuadEdge*> DelaunayTriangulation::Triangulate(std::vector<float2>& orderedPoints, uint64_t start, uint64_t end, bool vertical, bool dwyer)
{
    std::pair <QuadEdge*, QuadEdge*> extremities;

    // Base case where split left 2 vertices together, we form an edge out of them
    if (end - start == 2)
    {
        if (vertical)
        {
            std::sort(orderedPoints.begin() + start, orderedPoints.begin() + end, x_first);
        }
        else
        {
            std::sort(orderedPoints.begin() + start, orderedPoints.begin() + end, y_first);
        }
        QuadEdge* e = QuadEdge::MakeEdge(edges, orderedPoints[start], orderedPoints[end - 1]);
        extremities.first = e;
        extremities.second = e->m_sym;
        return extremities;
    }
    // Base case where split left 3 vertices together, we form an edge between p1, p2 and p2, p3 (recall that the points are X sorted), and connect the 2 edges
    if (end - start == 3)
    {
        if (vertical)
        {
            std::sort(orderedPoints.begin() + start, orderedPoints.begin() + end, x_first);
        }
        else
        {
            std::sort(orderedPoints.begin() + start, orderedPoints.begin() + end, y_first);
        }
        // Connect p1p2 and p2p3
        float2 p1, p2, p3;
        p1 = orderedPoints[start];
        p2 = orderedPoints[start + 1];
        p3 = orderedPoints[end - 1];
        QuadEdge* a = QuadEdge::MakeEdge(edges, p1, p2);
        QuadEdge* b = QuadEdge::MakeEdge(edges, p2, p3);
        QuadEdge::Splice(a->m_sym, b);

        // Closing the triangle
        // Case where p3 is on the right side of p1p2
        if (CCW(p1, p2, p3))
        {
            QuadEdge::Connect(edges, b, a);
            extremities.first = a;
            extremities.second = b->m_sym;
            return extremities;
        }
        // Case where p3 os on the left side of p1p2
        if (CCW(p1, p3, p2))
        {
            QuadEdge* c = QuadEdge::Connect(edges, b, a);
            extremities.first = c->m_sym;
            extremities.second = c;
            return extremities;
        }
        // Case where p1 p2 p3 are colinear
        extremities.first = a;
        extremities.second = b->m_sym;
        return extremities;
    }


    // Split points in the middle and recursively triangulate both parts
    uint64_t middle = (end - start + 1) / 2;
    if (vertical)
    {
        // Partition elements in two halves based on x value
        std::nth_element(
            orderedPoints.begin() + start,
            orderedPoints.begin() + start + middle,
            orderedPoints.begin() + end,
            x_first
        );
    }
    else
    {
        // Partition elements in two halves based on y value
        std::nth_element(
            orderedPoints.begin() + start,
            orderedPoints.begin() + start + middle,
            orderedPoints.begin() + end,
            y_first
        );
    }

    std::pair <QuadEdge*, QuadEdge*> leftHalf, rightHalf;
    QuadEdge* ldo, * ldi, * rdi, * rdo;
    if (dwyer)
    {
        leftHalf = Triangulate(orderedPoints, start, start + middle, !vertical);
        rightHalf = Triangulate(orderedPoints, start + middle, end, !vertical);
    }
    else
    {
        leftHalf = Triangulate(orderedPoints, start, start + middle, true, false);
        rightHalf = Triangulate(orderedPoints, start + middle, end, true, false);
    }
    ldo = leftHalf.first;
    ldi = leftHalf.second;
    rdi = rightHalf.first;
    rdo = rightHalf.second;

    // Rearrange the pointers to suit horizontal and vertical merging
    bool (*comparator)(const float2&, const float2&) = vertical ? x_first : y_first;
    if (dwyer)
    {
        if (vertical) 
        { 
            while (comparator(ldo->m_sym->m_onext->m_org, ldo->m_org)) 
            {
                ldo = ldo->m_sym->m_onext;
            }

            while (comparator(ldi->m_org, ldi->m_onext->m_sym->m_org)) 
            {
                ldi = ldi->m_onext->m_sym;
            }

            while (comparator(rdi->m_sym->m_onext->m_org, rdi->m_org)) 
            {
                rdi = rdi->m_sym->m_onext;
            }
            while (comparator(rdo->m_org, rdo->m_onext->m_sym->m_org)) 
            {
                rdo = rdo->m_onext->m_sym;
            }
        }
        else {

            while (comparator(ldo->m_oprev->m_sym->m_org, ldo->m_org)) 
            {
                ldo = ldo->m_oprev->m_sym;
            }

            while (comparator(ldi->m_org, ldi->m_sym->m_oprev->m_org)) {
                ldi = ldi->m_sym->m_oprev;
            }

            while (comparator(rdi->m_oprev->m_sym->m_org, rdi->m_org)) {
                rdi = rdi->m_oprev->m_sym;
            }
            while (comparator(rdo->m_org, rdo->m_sym->m_oprev->m_org)) {
                rdo = rdo->m_sym->m_oprev;
            }
        }
    }

    // Finds the lower tangent of left and right part of the triangulation, i.e the lowest edge that can connect the two distinct triangulations
    while (true)
    {
        if (ldi->LeftOf(rdi->m_org))
        {
            ldi = ldi->m_sym->m_oprev;
        }
        else if (rdi->RightOf(ldi->m_org))
        {
            rdi = rdi->m_sym->m_onext;
        }
        else
        {
            break;
        }
    }

    // Connects the cross edge between left and right part
    QuadEdge* basel = QuadEdge::Connect(edges, rdi->m_sym, ldi);
    if (ldi->m_org == ldo->m_org)
    {
        ldo = basel->m_sym;
    }
    if (rdi->m_org == rdo->m_org)
    {
        rdo = basel;
    }


    // Merge two parts by "stitching" them from bottom to top, deleting edges that become illegal in the process
    // The variable basel is the line that is perpendicular to y, and goes up to stitch the two parts together. We also need to keep its "right side" towards the top during the process, 
    // because anything below basel is considered complete, and everything above is yet to be stitched. Therefore it is important to have a way of knowing what is above and below. By keeping its right side 
    // facing the top, this is assured
    while (true)
    {
        QuadEdge* rcand = basel->m_oprev;
        QuadEdge* lcand = basel->m_sym->m_onext;
        bool vRcand = basel->RightOf(rcand->m_dest);
        bool vLcand = basel->RightOf(lcand->m_dest);

        // Locate the first left point that forms an empty circumcircle with basel and delete all left edges for which a point failed to pass this test in the process
        if (vLcand)
        {
            while (InCircle(basel->m_dest, basel->m_org, lcand->m_dest, lcand->m_onext->m_dest))
            {
                QuadEdge* temp = lcand->m_onext;
                QuadEdge::DeleteEdge(lcand);
                lcand = temp;
            }
        }

        // Same for right side
        if (vRcand)
        {
            while (InCircle(basel->m_dest, basel->m_org, rcand->m_dest, rcand->m_oprev->m_dest))
            {
                QuadEdge* temp = rcand->m_oprev;
                QuadEdge::DeleteEdge(rcand);
                rcand = temp;
            }
        }

        // Here, we succeded to find a left point and a right point that forms an empty circumcircle with basel
        vRcand = basel->RightOf(rcand->m_dest);
        vLcand = basel->RightOf(lcand->m_dest);

        // If none are on the right of the base, it means we reached the top and the stitching is complete
        if (!vRcand && !vLcand)
        {
            break;
        }

        // Otherwise, at least one of the point lies above basel (on its right side), so we need to decide which one will get connected to basel
        // If left point is'nt above basel, connect right point. If it is above basel, check if it is a better candidate than right point 
        if (!vLcand
            || (vRcand
                && InCircle(lcand->m_dest, lcand->m_org, rcand->m_org, rcand->m_dest)))
        {
            basel = QuadEdge::Connect(edges, rcand, basel->m_sym);
        }
        // Otherwise left side is best candidate, so we connect it to basel
        else
        {
            basel = QuadEdge::Connect(edges, basel->m_sym, lcand->m_sym);
        }
    }

    extremities.first = ldo;
    extremities.second = rdo;
    return extremities;
}

void DelaunayTriangulation::TriangulatePoints(std::vector<float2>& points, std::vector<Edge>& edgesResult)
{
    // Sort points by coordinates and remove duplicates
    InitData(points);

    // Computes Delaunay's triangulation, Dwyer's variation is alternating horizontal and vertical split, this allows less triangles deletion when stitching, but we also need to implement horizontal merge
    Triangulate(points, 0, points.size(), true, true);

    // Remove trash edges generated during triangulation and convert to lighter structure
    for (QuadEdge* quadEdge : edges)
    {
        if (quadEdge->m_data)
        {
            continue; 
        }
        edgesResult.push_back({quadEdge->m_org, quadEdge->m_dest, quadEdge->Length()}); 
    }
}

DelaunayTriangulation::~DelaunayTriangulation()
{
    // Delete allocated memory for edges of the graph
    for (QuadEdge* QuadEdge : edges)
    {
        if (nullptr == QuadEdge->m_sym)
        {
            delete QuadEdge->m_sym;
        }
        delete QuadEdge;
    }
}

double QuadEdge::Length()
{
    return sqrt(pow(m_dest.y - m_org.y, 2) + pow(m_dest.x - m_org.x, 2));
}