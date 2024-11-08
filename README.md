# DelaunayDivideAndConquer

Implementation of a Divide and Conquer strategy to build a Delaunay triangulation. The implemented method comes from Rex A. Dwyer ([1]), which is an improved version of the method described in [2] by Guibas and Stolfi.

# MST

An implementation of Kruskal's MST algorithm is also given in this repository. These two algorithms was initially used together to find the MST of a fully connected graph (Delaunay triangulation used to reduce number of edges before using Kruskal on reduced graph)

# Example
Here we can see in grey the Delaunay triangulation and in red the MST of the graph.
![image](https://github.com/user-attachments/assets/1dfe2798-68bb-4bdc-a6c9-4dc71cc80d39)

## References
[1] Dwyer RA. A simple divide-and-conquer algorithm for computing Delaunay triangulations in o(n log log n) expected time. Proceedings of the Second Annual Symposium on Computational Geometry, 1986; 276–284. </br>
[2] L. J. Guibas & J. Stolfi, "Primitives for the manipulation of gcneral subdivisions and the computation of Voronoi diagrams," ACM Trans. Graphics 4 (1985), 74-123. 
