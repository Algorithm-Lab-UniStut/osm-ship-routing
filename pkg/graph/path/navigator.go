package path

type Navigator interface {
	//SetGraph(g graph.Graph)
	GetPath(origin, destination int) ([]int, int)
}
