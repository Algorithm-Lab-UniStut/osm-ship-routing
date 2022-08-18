package path

import (
	"container/heap"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g            graph.Graph
	visitedNodes []bool
	searchSpace  []*DijkstraItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	useHeuristic bool            // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	origin       graph.NodeId    // the origin of the current search
	destination  graph.NodeId    // the distinaiton of the current search
}

func NewUniversalDijkstra(g graph.Graph, useHeuristic bool) *UniversalDijkstra {
	// heuristic is initially "nil"
	return &UniversalDijkstra{g: g, useHeuristic: useHeuristic}
}

func (d *UniversalDijkstra) InitializeSearch(origin, destination graph.NodeId) {
	d.visitedNodes = make([]bool, d.g.NodeCount())
	d.searchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.origin = origin
	d.destination = destination
}

func (d *UniversalDijkstra) SettleNode(node *DijkstraItem) {
	d.searchSpace[node.nodeId] = node
	d.visitedNodes[node.nodeId] = true
}

func (d *UniversalDijkstra) RelaxEdges(node *DijkstraItem, pq *MinPath) {
	for _, arc := range d.g.GetArcsFrom(node.nodeId) {
		successor := arc.Destination()
		if d.searchSpace[successor] == nil {
			cost := node.distance + arc.Cost()
			heuristic := 0
			if d.useHeuristic {
				heuristic = d.estimatedDistance(successor, d.destination)
			}
			nextNode := NewDijkstraItem(successor, cost, node.nodeId, heuristic)
			d.searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedPriority := node.distance + arc.Cost() + d.searchSpace[successor].heuristic; updatedPriority < d.searchSpace[successor].Priority() {
				pq.update(d.searchSpace[successor], node.distance+arc.Cost())
				d.searchSpace[successor].predecessor = node.nodeId
			}
		}
	}
}

func (dijkstra *UniversalDijkstra) GetShortestPath(origin, destination graph.NodeId) int {
	dijkstra.InitializeSearch(origin, destination)
	heuristic := 0
	if dijkstra.useHeuristic {
		heuristic = dijkstra.estimatedDistance(origin, dijkstra.destination)
	}
	pq := NewMinPath(NewDijkstraItem(origin, 0, -1, heuristic))

	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		dijkstra.SettleNode(currentNode)

		if destination != -1 && currentNode.nodeId == destination {
			break
		}

		dijkstra.RelaxEdges(currentNode, pq)
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		return 0
	}
	if dijkstra.searchSpace[destination] == nil {
		// no valid path found
		return -1
	}
	length := dijkstra.searchSpace[destination].distance
	return length
}

func (dijkstra *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	dijkstra.useHeuristic = useHeuristic
}

func (dijkstra *UniversalDijkstra) GetPath(origin, destination int) ([]int, int) {
	length := dijkstra.GetShortestPath(origin, destination)
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0), 0
	}
	if length == -1 {
		// no path found
		return make([]int, 0), length
	}
	path := dijkstra.extractPath(origin, destination)

	return path, length
}

func (d *UniversalDijkstra) extractPath(origin, destination int) []int {
	path := make([]int, 0)
	for nodeId := destination; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
		path = append(path, nodeId)
	}
	// reverse path (to create the correct direction)
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

func (d *UniversalDijkstra) estimatedDistance(originNodeId, destinationNodeId int) int {
	origin := d.g.GetNode(originNodeId)
	destination := d.g.GetNode(destinationNodeId)
	//originPoint := geo.NewPoint(origin.Point.X, origin.Point.Y) // TODO: access point via node
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return originPoint.IntHaversine(destinationPoint)
}
