package path

import "github.com/natevvv/osm-ship-routing/pkg/graph"

type Navigator interface {
	//SetGraph(g graph.Graph)
	GetPath(origin, destination int) ([]int, int)
	GetNodes() []graph.Node
	GetSearchSpace() []graph.Node
}
