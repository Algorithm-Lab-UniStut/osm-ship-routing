package graph

func FindShortestPath(g Graph, origin, destination int) ([]int, int) {
	navigator := BidirectionalDijkstra{}
	path, length := navigator.ShortestPath(g, origin, destination)
	return path, length
}
