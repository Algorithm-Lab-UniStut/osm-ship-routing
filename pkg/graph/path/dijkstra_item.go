package path

import (
	"fmt"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

type Direction int

const (
	FORWARD  Direction = iota
	BACKWARD Direction = iota
)

func (d Direction) String() string {
	if d == FORWARD {
		return "FORWARD"
	}
	if d == BACKWARD {
		return "BACKWARD"
	}
	return "INVALID"
}

// implements queue.Priorizable
type DijkstraItem struct {
	NodeId          graph.NodeId // node id of this item in the graph
	distance        int          // distance to origin of this node
	heuristic       int          // estimated distance from node to destination
	predecessor     graph.NodeId // node id of the predecessor
	index           int          // internal usage
	searchDirection Direction    // search direction (useful for bidirectional search)
}

func NewDijkstraItem(nodeId graph.NodeId, distance int, predecessor graph.NodeId, heuristic int, searchDirection Direction) *DijkstraItem {
	if searchDirection != BACKWARD && searchDirection != FORWARD {
		panic("bad direction")
	}
	return &DijkstraItem{NodeId: nodeId, distance: distance, predecessor: predecessor, index: -1, heuristic: heuristic, searchDirection: searchDirection}
}

// TODO check pointer receivers
func (d DijkstraItem) Priority() int       { return d.distance + d.heuristic }
func (d DijkstraItem) Index() int          { return d.index }
func (d *DijkstraItem) SetIndex(index int) { d.index = index }
func (d *DijkstraItem) String() string {
	return fmt.Sprintf("%v: %v, %v\n", d.index, d.NodeId, d.Priority())
}
