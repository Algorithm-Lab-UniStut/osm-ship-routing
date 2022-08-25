package path

import (
	"container/heap"
	"fmt"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g                       graph.Graph
	visitedNodes            map[graph.NodeId]bool // Set which contains the visited nodes (only true values)
	backwardVisitedNodes    map[graph.NodeId]bool // Set which contains the visited nodes of the backward search
	searchSpace             []*DijkstraItem       // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	backwardSearchSpace     []*DijkstraItem       // search space for the backward search
	origin                  graph.NodeId          // the origin of the current search
	destination             graph.NodeId          // the distination of the current search
	pathLength              int                   // length of the computed path (-1 if no path found)
	useHeuristic            bool                  // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional           bool                  // flag indicating if search shuld be done from both sides
	bidirectionalConnection *BidirectionalConnection
	considerArcFlags        bool
	costUpperBound          int
	maxNumSettledNodes      int
	debugLevel              int
}

type BidirectionalConnection struct {
	nodeId      graph.NodeId
	predecessor graph.NodeId
	successor   graph.NodeId
	distance    int
}

func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	return &UniversalDijkstra{g: g, visitedNodes: make(map[graph.NodeId]bool), backwardVisitedNodes: make(map[graph.NodeId]bool), searchSpace: make([]*DijkstraItem, g.NodeCount()), backwardSearchSpace: make([]*DijkstraItem, g.NodeCount()), bidirectionalConnection: nil, costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, pathLength: -1}
}

func NewBidirectionalConnection(nodeId, predecessor, successor graph.NodeId, distance int) *BidirectionalConnection {
	return &BidirectionalConnection{nodeId: nodeId, predecessor: predecessor, successor: successor, distance: distance}
}

func (d *UniversalDijkstra) InitializeSearch(origin, destination graph.NodeId) {
	if d.debugLevel >= 1 {
		fmt.Printf("visited nodes: %v\n", d.visitedNodes)
	}
	for nodeId := range d.visitedNodes {
		d.searchSpace[nodeId] = nil
		for _, arc := range d.g.GetArcsFrom(nodeId) {
			// also reset the potential other enqueued nodes
			d.searchSpace[arc.Destination()] = nil
		}
	}
	for nodeId := range d.backwardVisitedNodes {
		d.backwardSearchSpace[nodeId] = nil
		for _, arc := range d.g.GetArcsFrom(nodeId) {
			// also reset the potential other enqueued nodes
			d.backwardSearchSpace[arc.Destination()] = nil
		}
	}
	d.visitedNodes = make(map[graph.NodeId]bool)
	d.backwardVisitedNodes = make(map[graph.NodeId]bool)
	d.origin = origin
	d.destination = destination
	if d.bidirectional {
		d.bidirectionalConnection = nil
	}
}

func (d *UniversalDijkstra) SettleNode(node *DijkstraItem) {
	if d.debugLevel >= 1 {
		fmt.Printf("Settle Node %v\n", node.NodeId)
	}
	//fmt.Printf("Settling %v, direction: %v\n", node.NodeId, node.searchDirection)
	searchSpace, visitedNodes := d.searchSpace, d.visitedNodes
	if node.searchDirection == BACKWARD {
		searchSpace, visitedNodes = d.backwardSearchSpace, d.backwardVisitedNodes
	}
	searchSpace[node.NodeId] = node
	visitedNodes[node.NodeId] = true
}

func (d *UniversalDijkstra) IsFullSettled(nodeId graph.NodeId) bool {
	visited := d.visitedNodes[nodeId]
	backwardVisited := d.backwardVisitedNodes[nodeId]
	return visited && backwardVisited
}

func (dijkstra *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
	dijkstra.InitializeSearch(origin, destination)
	if dijkstra.useHeuristic && dijkstra.bidirectional {
		panic("AStar doesn't work bidirectional so far.")
	}
	if destination == -1 && dijkstra.bidirectional {
		panic("Can't use bidirectional search with no specified destination")
	}
	heuristic := 0
	if dijkstra.useHeuristic {
		heuristic = dijkstra.g.EstimateDistance(origin, dijkstra.destination)
	}
	originItem := NewDijkstraItem(origin, 0, -1, heuristic, FORWARD)
	pq := NewMinPath(originItem)
	// Initialize
	dijkstra.SettleNode(originItem)

	// for bidirectional algorithm
	if dijkstra.bidirectional {
		destinationItem := NewDijkstraItem(destination, 0, -1, heuristic, BACKWARD)
		heap.Push(pq, destinationItem)
		// Initialize
		dijkstra.SettleNode(destinationItem)
	}

	numSettledNodes := 0
	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		dijkstra.SettleNode(currentNode)
		numSettledNodes++
		if dijkstra.costUpperBound < currentNode.Priority() || dijkstra.maxNumSettledNodes < numSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			dijkstra.pathLength = -1
			return -1
		}
		if dijkstra.bidirectionalConnection != nil && dijkstra.IsFullSettled(currentNode.NodeId) {
			// node with lowest priority is the current connection node
			// -> every edge increases cost/priority
			// -> this has to be the shortest path --> wrong, if one edge is (really) long
			// Correction: current node (with lowest possible distance) is already greater than connection distance
			// -> connection has to be the lowest one
			// -> Wrong, this condition just made a "normal" Dijkstra out of the Bidirectional Dijkstra
			// Correction: check if the node was already settled in both directions, this is a connection
			// the connection item contains the information, which one is the real connection (this has not to be the current one)
			break
		}

		if destination != -1 {
			if currentNode.searchDirection == FORWARD && currentNode.NodeId == destination {
				break
			} else if dijkstra.bidirectional && currentNode.searchDirection == BACKWARD && currentNode.NodeId == origin {
				// not necessary?
				break
			}
		}

		dijkstra.RelaxEdges(currentNode, pq)
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		dijkstra.pathLength = 0
		return 0
	}

	if dijkstra.bidirectional {
		if dijkstra.bidirectionalConnection == nil {
			// no valid path found
			dijkstra.pathLength = -1
			return -1
		}
		length := dijkstra.bidirectionalConnection.distance
		dijkstra.pathLength = length
		return length
	}

	if dijkstra.searchSpace[destination] == nil {
		// no valid path found
		dijkstra.pathLength = -1
		return -1
	}
	length := dijkstra.searchSpace[destination].distance
	dijkstra.pathLength = length
	return length
}

func (d *UniversalDijkstra) RelaxEdges(node *DijkstraItem, pq *MinPath) {
	if d.debugLevel >= 1 {
		fmt.Printf("Relax Edges for node %v\n", node.NodeId)
	}
	for _, arc := range d.g.GetArcsFrom(node.NodeId) {
		if d.considerArcFlags && !arc.ArcFlag() {
			// ignore this arc
			continue
		}

		if d.debugLevel >= 1 {
			fmt.Printf("Relax Edge %v -> %v", node.NodeId, arc.Destination())
		}
		successor := arc.Destination()
		searchSpace, inverseSearchSpace := d.searchSpace, d.backwardSearchSpace
		if node.searchDirection == BACKWARD {
			searchSpace, inverseSearchSpace = d.backwardSearchSpace, d.searchSpace
		}
		if d.bidirectional && inverseSearchSpace[successor] != nil {
			// store potential connection node, needed for later
			// this is a "real" copy, not just a pointer since it get changed now
			connection := inverseSearchSpace[successor]
			connectionPredecessor := node.NodeId
			connectionSuccessor := connection.predecessor
			var con *BidirectionalConnection
			if node.searchDirection == FORWARD {
				con = NewBidirectionalConnection(connection.NodeId, connectionPredecessor, connectionSuccessor, node.distance+arc.Cost()+connection.distance)
			} else {
				con = NewBidirectionalConnection(connection.NodeId, connectionSuccessor, connectionPredecessor, node.distance+arc.Cost()+connection.distance)
			}
			if d.bidirectionalConnection == nil || con.distance < d.bidirectionalConnection.distance {
				d.bidirectionalConnection = con
			}
		}
		if searchSpace[successor] == nil {
			cost := node.distance + arc.Cost()
			heuristic := 0
			if d.useHeuristic {
				heuristic = d.g.EstimateDistance(successor, d.destination)
			}
			nextNode := NewDijkstraItem(successor, cost, node.NodeId, heuristic, node.searchDirection)
			searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedPriority := node.distance + arc.Cost() + searchSpace[successor].heuristic; updatedPriority < searchSpace[successor].Priority() {
				pq.update(searchSpace[successor], node.distance+arc.Cost())
				searchSpace[successor].predecessor = node.NodeId
				//searchSpace[successor].searchDirection = node.searchDirection
			}
		}
	}
}

func (dijkstra *UniversalDijkstra) GetPath(origin, destination int) []int {
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0)
	}
	if dijkstra.pathLength == -1 {
		// no path found
		return make([]int, 0)
	}
	path := make([]int, 0)
	if dijkstra.bidirectional {
		if dijkstra.debugLevel >= 1 {
			fmt.Printf("con: %v, pre: %v, suc: %v\n", dijkstra.bidirectionalConnection.nodeId, dijkstra.bidirectionalConnection.predecessor, dijkstra.bidirectionalConnection.successor)
		}
		for nodeId := dijkstra.bidirectionalConnection.predecessor; nodeId != -1; nodeId = dijkstra.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseIntInPlace(path)
		path = append(path, dijkstra.bidirectionalConnection.nodeId)
		for nodeId := dijkstra.bidirectionalConnection.successor; nodeId != -1; nodeId = dijkstra.backwardSearchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	} else {
		for nodeId := destination; nodeId != -1; nodeId = dijkstra.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		// reverse path (to create the correct direction)
		slice.ReverseIntInPlace(path)
	}

	return path
}

func (d *UniversalDijkstra) GetSearchSpace() []*DijkstraItem {
	searchSpace := make([]*DijkstraItem, len(d.visitedNodes)+len(d.backwardVisitedNodes))
	i := 0
	for nodeId := range d.visitedNodes {
		searchSpace[i] = d.searchSpace[nodeId]
		i++
	}
	for nodeId := range d.backwardVisitedNodes {
		searchSpace[i] = d.backwardSearchSpace[nodeId]
		i++
	}
	return searchSpace
}

func (dijkstra *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	dijkstra.useHeuristic = useHeuristic
}

func (dijkstra *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	dijkstra.bidirectional = bidirectional
}

func (d *UniversalDijkstra) SetConsiderArcFlags(considerArcFlags bool) {
	d.considerArcFlags = considerArcFlags
}

func (d *UniversalDijkstra) SetCostUpperBound(costUpperBound int) {
	d.costUpperBound = costUpperBound
}

func (d *UniversalDijkstra) SetMaxNumSettledNodes(maxNumSettledNodes int) {
	d.maxNumSettledNodes = maxNumSettledNodes
}

func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}
