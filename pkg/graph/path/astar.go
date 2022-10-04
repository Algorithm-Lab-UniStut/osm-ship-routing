package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type AStarPriorityQueueItem struct {
	queue.Item      // "inherited" priority distinguishes the estimated distance from this node to destination
	Distance   int  // real distance from origin to this node
	Settled    bool // not sure
}

type AStarPriorityQueue []*AStarPriorityQueueItem

type AStar struct {
	g graph.Graph
}

type ImprovedAStar struct {
	g        graph.Graph
	dijkstra *UniversalDijkstra
}

func NewImproveAStar(g graph.Graph, dijkstra *UniversalDijkstra) *ImprovedAStar {
	return &ImprovedAStar{g: g, dijkstra: dijkstra}
}

func (astar *ImprovedAStar) ComputeShortestPath(origin, destination int) int {
	astar.dijkstra.SetUseHeuristic(true)
	length := astar.dijkstra.ComputeShortestPath(origin, destination)
	return length
}

func (astar *ImprovedAStar) GetPath(origin, destination int) []int {
	// calculate heuristic
	// not needed. it is more performantly to compute this on the fly
	/*
		heuristic := make([]int, astar.g.NodeCount(), astar.g.NodeCount())
		destNode := astar.g.GetNode(destination)
		destPoint := geo.NewPoint(destNode.Lat, destNode.Lon)
		for i := 0; i < astar.g.NodeCount(); i++ {
			sourceNode := astar.g.GetNode(i)
			sourcePoint := geo.NewPoint(sourceNode.Lat, sourceNode.Lon)
			h := sourcePoint.IntHaversine(destPoint)
			heuristic[i] = h
		}
	*/

	// execute query
	path := astar.dijkstra.GetPath(origin, destination)
	return path
}

func NewAStarPriorityQueueItem(id, priority, predecessor, distance int) *AStarPriorityQueueItem {
	//pqi := AStarPriorityQueueItem{PriorityQueueItem{ItemId: id, Priority: priority, Predecessor: predecessor, Index: index}, distance, settled}
	pqi := AStarPriorityQueueItem{Item: *queue.NewQueueItem(id, priority, predecessor) /*{ItemId: id, Priority: priority, Predecessor: predecessor, Index: index}*/, Distance: distance, Settled: false}
	return &pqi
}

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

func NewAStar(g graph.Graph) AStar {
	return AStar{g: g}
}

func (a AStar) estimatedDistance(originNodeId, destinationNodeId int) int {
	origin := a.g.GetNode(originNodeId)
	destination := a.g.GetNode(destinationNodeId)
	return origin.IntHaversine(destination)
}

func (a AStar) GetPath(origin, destination int) ([]int, int) {
	dijkstraItems := make([]*AStarPriorityQueueItem, a.g.NodeCount())
	originItem := NewAStarPriorityQueueItem(origin, 0, -1, 0) //{PriorityQueueItem: queue.PriorityQueueItem{ItemId: origin, Priority: 0, Predecessor: -1, Index: -1}}
	dijkstraItems[origin] = originItem

	pq := make(AStarPriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, dijkstraItems[origin])

	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*AStarPriorityQueueItem)
		currentNodeId := currentPqItem.ItemId

		for _, arc := range a.g.GetArcsFrom(currentNodeId) {
			successor := arc.Destination()

			if dijkstraItems[successor] == nil {
				newDistance := currentPqItem.Distance + arc.Cost()
				newPriority := newDistance + a.estimatedDistance(successor, destination)
				pqItem := NewAStarPriorityQueueItem(successor, newPriority, currentNodeId, newDistance) //{PriorityQueueItem: queue.PriorityQueueItem{ItemId: successor, Priority: newPriority, Predecessor: currentNodeId, Index: -1}, Distance: newDistance}
				dijkstraItems[successor] = pqItem
				heap.Push(&pq, pqItem)
			} else {
				if updatedPriority := currentPqItem.Distance + arc.Cost() + a.estimatedDistance(successor, destination); updatedPriority < dijkstraItems[successor].Priority {
					pq.update(dijkstraItems[successor], updatedPriority, currentPqItem.Distance+arc.Cost())
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

func (a AStar) GetSearchSpace() []*DijkstraItem {
	panic("not implemented")
}
