package path

import "github.com/natevvv/osm-ship-routing/pkg/graph"

type Navigator interface {
	GetPath(origin, destination int) []int           // Get the path of a previous computation. This contains the nodeIds which lie on the path from source to destination
	ComputeShortestPath(origin, destination int) int // Compute the shortest path from the origin to the destination
	GetSearchSpace() []*DijkstraItem                 // Returns the search space of a previous computation. This contains all items which were settled.
	GetPqPops() int                                  // Returns the amount of priority queue/heap pops which werer performed during the search
	GetPqUpdates() int                               // Get the number of pq updates
	GetEdgeRelaxations() int                         // Get the number of relaxed edges
	GetRelaxationAttempts() int                      // Get the number of attempted edge relaxations (some may early terminated)
	GetGraph() graph.Graph                           // Get the used graph
}
