package graph

import (
	"container/heap"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
)

type AStarPriorityQueueItem struct {
	PriorityQueueItem      // "inherited" priority distinguishes the estimated distance from this node to destination
	distance          int  // real distance from origin to this node
	settled           bool // not sure
}

func NewAStarPriorityQueueItem(id, priority, predecessor, index, distance int, settled bool) AStarPriorityQueueItem {
	//pqi := AStarPriorityQueueItem{PriorityQueueItem{itemId: id, priority: priority, predecessor: predecessor, index: index}, distance, settled}
	pqi := AStarPriorityQueueItem{PriorityQueueItem: PriorityQueueItem{itemId: id, priority: priority, predecessor: predecessor, index: index}, distance: distance, settled: settled}
	return pqi
}

type AStarPriorityQueue []*AStarPriorityQueueItem

func (h AStarPriorityQueue) Len() int { return len(h) }

func (h AStarPriorityQueue) Less(i, j int) bool {
	return h[i].priority < h[j].priority
}

func (h AStarPriorityQueue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index, h[j].index = i, j
}

func (h *AStarPriorityQueue) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*AStarPriorityQueueItem)
	pqItem.index = n
	*h = append(*h, pqItem)
}

func (h *AStarPriorityQueue) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *AStarPriorityQueue) update(pqItem *AStarPriorityQueueItem, newPriority int, newDistance int) {
	pqItem.priority = newPriority
	pqItem.distance = newDistance
	heap.Fix(h, pqItem.index)
}

type AStar struct {
}

func NewAStar(g Graph) AStar {
	return AStar{}
}

func estimatedDistance(g Graph, originNodeId, destinationNodeId int) int {
	origin := g.GetNode(originNodeId)
	destination := g.GetNode(destinationNodeId)
	//originPoint := geo.NewPoint(origin.Point.X, origin.Point.Y) // TODO: access point via node
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return originPoint.IntHaversine(destinationPoint)
}

func (a AStar) ShortestPath(g Graph, origin, destination int) ([]int, int) {
	dijkstraItems := make([]*AStarPriorityQueueItem, g.NodeCount(), g.NodeCount())
	originItem := AStarPriorityQueueItem{PriorityQueueItem: PriorityQueueItem{itemId: origin, priority: 0, predecessor: -1, index: -1}}
	dijkstraItems[origin] = &originItem

	pq := make(AStarPriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, dijkstraItems[origin])

	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*AStarPriorityQueueItem)
		currentNodeId := currentPqItem.itemId

		for _, edge := range g.GetEdgesFrom(currentNodeId) {
			successor := edge.To

			if dijkstraItems[successor] == nil {
				newDistance := currentPqItem.distance + edge.Distance
				newPriority := newDistance + estimatedDistance(g, successor, destination)
				pqItem := AStarPriorityQueueItem{PriorityQueueItem: PriorityQueueItem{itemId: successor, priority: newPriority, predecessor: currentNodeId, index: -1}, distance: newDistance}
				dijkstraItems[successor] = &pqItem
				heap.Push(&pq, &pqItem)
			} else {
				if updatedPriority := currentPqItem.distance + edge.Distance + estimatedDistance(g, successor, destination); updatedPriority < dijkstraItems[successor].priority {
					pq.update(dijkstraItems[successor], updatedPriority, currentPqItem.distance+edge.Distance)
					dijkstraItems[successor].predecessor = currentNodeId
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
		length = dijkstraItems[destination].priority
		for nodeId := destination; nodeId != -1; nodeId = dijkstraItems[nodeId].predecessor {
			path = append([]int{nodeId}, path...)
		}
	}

	return path, length
}
