package path

import (
	"fmt"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

// implement queue.Priorizable
type OrderItem struct {
	nodeId             graph.NodeId
	edgeDifference     int
	processedNeighbors int
	index              int
}

func (o *OrderItem) NodeId() graph.NodeId { return o.nodeId }
func (o *OrderItem) Priority() int        { return o.edgeDifference + o.processedNeighbors }
func (o *OrderItem) Index() int           { return o.index }
func (o *OrderItem) SetIndex(i int)       { o.index = i }
func (o *OrderItem) String() string {
	return fmt.Sprintf("%v: %v, %v\n", o.index, o.nodeId, o.Priority())
}

func NewOrderItem(nodeId graph.NodeId) *OrderItem {
	return &OrderItem{nodeId: nodeId, index: -1}
}
