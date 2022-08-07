package path

import "github.com/natevvv/osm-ship-routing/pkg/graph"

func FindShortestPath(g graph.Graph, origin, destination int) ([]int, int) {
	navigator := BidirectionalDijkstra{}
	path, length := navigator.ShortestPath(g, origin, destination)
	return path, length
}
