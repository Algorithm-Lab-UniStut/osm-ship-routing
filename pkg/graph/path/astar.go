package path

import (
	"container/heap"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type AStarPriorityQueueItem struct {
	queue.PriorityQueueItem      // "inherited" priority distinguishes the estimated distance from this node to destination
	Distance                int  // real distance from origin to this node
	Settled                 bool // not sure
}

func NewAStarPriorityQueueItem(id, priority, predecessor, index, distance int, settled bool) AStarPriorityQueueItem {
	//pqi := AStarPriorityQueueItem{PriorityQueueItem{ItemId: id, Priority: priority, Predecessor: predecessor, Index: index}, distance, settled}
	pqi := AStarPriorityQueueItem{PriorityQueueItem: queue.PriorityQueueItem{ItemId: id, Priority: priority, Predecessor: predecessor, Index: index}, Distance: distance, Settled: settled}
	return pqi
}

type AStarPriorityQueue []*AStarPriorityQueueItem

func (h AStarPriorityQueue) Len() int { return len(h) }

func (h AStarPriorityQueue) Less(i, j int) bool {
	return h[i].Priority < h[j].Priority
}

func (h AStarPriorityQueue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].Index, h[j].Index = i, j
}

func (h *AStarPriorityQueue) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*AStarPriorityQueueItem)
	pqItem.Index = n
	*h = append(*h, pqItem)
}

func (h *AStarPriorityQueue) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.Index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *AStarPriorityQueue) update(pqItem *AStarPriorityQueueItem, newPriority int, newDistance int) {
	pqItem.Priority = newPriority
	pqItem.Distance = newDistance
	heap.Fix(h, pqItem.Index)
}

type AStar struct {
}

func NewAStar(g graph.Graph) AStar {
	return AStar{}
}

func estimatedDistance(g graph.Graph, originNodeId, destinationNodeId int) int {
	origin := g.GetNode(originNodeId)
	destination := g.GetNode(destinationNodeId)
	//originPoint := geo.NewPoint(origin.Point.X, origin.Point.Y) // TODO: access point via node
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return originPoint.IntHaversine(destinationPoint)
}

func (a AStar) GetPath(g graph.Graph, origin, destination int) ([]int, int) {
	dijkstraItems := make([]*AStarPriorityQueueItem, g.NodeCount(), g.NodeCount())
	originItem := AStarPriorityQueueItem{PriorityQueueItem: queue.PriorityQueueItem{ItemId: origin, Priority: 0, Predecessor: -1, Index: -1}}
	dijkstraItems[origin] = &originItem

	pq := make(AStarPriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, dijkstraItems[origin])

	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*AStarPriorityQueueItem)
		currentNodeId := currentPqItem.ItemId

		for _, edge := range g.GetEdgesFrom(currentNodeId) {
			successor := edge.To

			if dijkstraItems[successor] == nil {
				newDistance := currentPqItem.Distance + edge.Distance
				newPriority := newDistance + estimatedDistance(g, successor, destination)
				pqItem := AStarPriorityQueueItem{PriorityQueueItem: queue.PriorityQueueItem{ItemId: successor, Priority: newPriority, Predecessor: currentNodeId, Index: -1}, Distance: newDistance}
				dijkstraItems[successor] = &pqItem
				heap.Push(&pq, &pqItem)
			} else {
				if updatedPriority := currentPqItem.Distance + edge.Distance + estimatedDistance(g, successor, destination); updatedPriority < dijkstraItems[successor].Priority {
					pq.update(dijkstraItems[successor], updatedPriority, currentPqItem.Distance+edge.Distance)
					dijkstraItems[successor].Predecessor = currentNodeId
				}
			}
		}

		if currentNodeId == destination {
			break
		}
	}

	length := -1
	path := make([]int, 0)

	if dijkstraItems[destination] != nil {
		length = dijkstraItems[destination].Priority
		for nodeId := destination; nodeId != -1; nodeId = dijkstraItems[nodeId].Predecessor {
			path = append([]int{nodeId}, path...)
		}
	}

	return path, length
}
