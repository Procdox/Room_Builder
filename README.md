# Room_Builder

Room_Builder is a plugin for Unreal Engine to support the creation of grid-less procedural layouts. In it's current form, it is able to allocate and resolve 2 dimensional geometry. It then allows local traversal through the structure by connectivity of regions.
# As it is
The project utilizes python lists only for storing data within either format. Dense algorithms operate exactly as one would naively expect. All sparse operations operate by iterating across rows nested in iterating across columns. For multiplication, the transform is calculated for the right matrix, then both the left and right are iterated across simultaneously.

This allows greedier methods, or isolated local methods to be used to generate geometry, then still be able traverse the larger structure as it interacts with other regions.

# What will be

The project will provide ways of extracting the connectivity graph of regions, as well as labeling the nodes (regions) and connections (edges). This could be fed into algorithms to refine the details or regiuons or how they connect.

# What I'm working on

* A system for identifying connected parrallel edges, to allow for allignment adjustment, elliminating small regions or misallignments.
* A generic typing system for regions and edges, other than the integer marks.
* Two known errors, one involving external but touching sub-allocation regions, one involving the triangulation of region floors.
