package path

import "github.com/natevvv/osm-ship-routing/pkg/graph"

type Navigator interface {
	//SetGraph(g graph.Graph)
	GetPath(origin, destination int) []int
	ComputeShortestPath(origin, destination int) int
	GetSearchSpace() []*DijkstraItem
	GetPqPops() int
	GetPqUpdates() int
	GetEdgeRelaxations() int
	GetRelaxationAttempts() int
	GetGraph() graph.Graph
}
