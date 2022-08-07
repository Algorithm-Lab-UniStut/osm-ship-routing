package path

import "github.com/natevvv/osm-ship-routing/pkg/graph"

type Navigator interface {
	GetPath(g graph.Graph, origin, destination int) ([]int, int)
}
