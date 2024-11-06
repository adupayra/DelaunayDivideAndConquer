    
#ifndef HELPERS_H
#define HELPERS_H

/*
* This file contains helper functions and structures that might be useful for multiple classes and does't particularly belong to one class semantically speaking
*/

struct float2
{
    float x;
    float y;
    uint64_t ID;

    float2(float X, float Y)
        : x(X)
        , y(Y)
        , ID(0)
    {}
    float2()
        : x(0)
        , y(0)
        , ID(0)
    {}

    bool operator==(float2 v)
    {
        return this->x == v.x && this->y == v.y;
    }

    bool operator < (const float2& v) const
    {
        if (this->x == v.x)
        {
            return this->y < v.y;
        }
        return this->x < v.x;
    }

    void print()
    {
        printf("(%f, %f)\n", x, y);
    }
};


struct Edge
{
    float2 start;
    float2 end;
    double length;
};

// Utils function to export a list of edges to CSV. Used during debugging to display triangulation and MST easily on Python
void ExportToCsv(std::string filename, const std::vector<Edge>& edgesToExport);

#endif // HELPERS_H