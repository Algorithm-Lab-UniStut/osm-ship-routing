package path

import (
	"fmt"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

type Direction bool

const (
	FORWARD  Direction = false
	BACKWARD Direction = true
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
	nodeId          graph.NodeId // node id of this item in the graph
	distance        int          // distance to origin of this node
	heuristic       int          // estimated distance from node to destination
	predecessor     graph.NodeId // node id of the predecessor
	index           int          // internal usage
	searchDirection Direction    // search direction (useful for bidirectional search)
}

func NewDijkstraItem(nodeId graph.NodeId, distance int, predecessor graph.NodeId, heuristic int, searchDirection Direction) *DijkstraItem {
	return &DijkstraItem{nodeId: nodeId, distance: distance, predecessor: predecessor, index: -1, heuristic: heuristic, searchDirection: searchDirection}
}

func (item *DijkstraItem) NodeId() graph.NodeId { return item.nodeId }
func (item *DijkstraItem) Priority() int        { return item.distance + item.heuristic }
func (item *DijkstraItem) Index() int           { return item.index }
func (item *DijkstraItem) SetIndex(index int)   { item.index = index }
func (item *DijkstraItem) String() string {
	return fmt.Sprintf("%v: %v, %v\n", item.index, item.nodeId, item.Priority())
}
