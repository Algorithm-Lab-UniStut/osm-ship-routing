package graph

type Navigator interface {
	ShortestPath(g Graph, origin, destination int) ([]int, int)
}
