package path

import (
	"container/heap"
	"fmt"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

// UniversalDijkstra implements various path finding algorithms which all are based on Dijkstra.
// it can be used for plain Dijsktra, Bidirectional search, A* (and maybe more will come).
// Implements the Navigator Interface.
type UniversalDijkstra struct {
	// check if pointers are needed/better
	g                       graph.Graph
	visitedNodes            []bool          // Set which contains the visited nodes (only true values)
	backwardVisitedNodes    []bool          // Set which contains the visited nodes of the backward search
	searchSpace             []*DijkstraItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	backwardSearchSpace     []*DijkstraItem // search space for the backward search
	origin                  graph.NodeId    // the origin of the current search
	destination             graph.NodeId    // the distination of the current search
	pathLength              int             // length of the computed path (-1 if no path found)
	useHeuristic            bool            // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional           bool            // flag indicating if search should be done from both sides
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

// Create a new Dijkstra instance with the given graph g
func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	return &UniversalDijkstra{g: g, visitedNodes: make([]bool, g.NodeCount()), backwardVisitedNodes: make([]bool, g.NodeCount()), searchSpace: make([]*DijkstraItem, g.NodeCount()), backwardSearchSpace: make([]*DijkstraItem, g.NodeCount()), bidirectionalConnection: nil, costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, pathLength: -1}
}

// Creates a new item which describes a connection between a forward and a backward search
// The nodeId is the id of the node which connects both searches, the predecessor represents the node of the forward search and the successor the node of the backward search.
// The distance is the length needed from both searches to reach this node
func NewBidirectionalConnection(nodeId, predecessor, successor graph.NodeId, distance int) *BidirectionalConnection {
	return &BidirectionalConnection{nodeId: nodeId, predecessor: predecessor, successor: successor, distance: distance}
}

// Compute the shortest path from the origin to the destination.
// It returns the length of the found path
// If no path was found, it returns -1
// If the path to all possible target is calculated (set target to -1), it returns 0
func (dijkstra *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
	dijkstra.initializeSearch(origin, destination)
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
	dijkstra.settleNode(originItem)

	// for bidirectional algorithm
	if dijkstra.bidirectional {
		destinationItem := NewDijkstraItem(destination, 0, -1, 0, BACKWARD)
		heap.Push(pq, destinationItem)
		// Initialize
		dijkstra.settleNode(destinationItem)
	}

	numSettledNodes := 0
	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		dijkstra.settleNode(currentNode)
		numSettledNodes++
		if dijkstra.costUpperBound < currentNode.Priority() || dijkstra.maxNumSettledNodes < numSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			dijkstra.pathLength = -1
			return -1
		}
		if dijkstra.bidirectionalConnection != nil && dijkstra.isFullySettled(currentNode.NodeId) {
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

		dijkstra.relaxEdges(currentNode, pq)
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

// Get the path of a previous computation. This contains the nodeIds which lie on the path from source to destination
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

// Returns the search space of a previous computation. This contains all items which were settled.
func (d *UniversalDijkstra) GetSearchSpace() []*DijkstraItem {
	visitedNodes := 0
	for _, visited := range d.visitedNodes {
		if visited {
			visitedNodes++
		}
	}
	for _, visited := range d.backwardVisitedNodes {
		if visited {
			visitedNodes++
		}
	}
	searchSpace := make([]*DijkstraItem, visitedNodes)
	i := 0
	for nodeId, visited := range d.visitedNodes {
		if visited {
			searchSpace[i] = d.searchSpace[nodeId]
			i++
		}
	}
	for nodeId, visited := range d.backwardVisitedNodes {
		if visited {
			searchSpace[i] = d.backwardSearchSpace[nodeId]
			i++
		}
	}
	return searchSpace
}

// Initialize a new search
// This resets the search space and visited nodes (and all other leftovers of a previous search)
func (d *UniversalDijkstra) initializeSearch(origin, destination graph.NodeId) {
	if d.debugLevel >= 1 {
		fmt.Printf("visited nodes: %v\n", d.visitedNodes)
	}
	d.searchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.backwardSearchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.visitedNodes = make([]bool, d.g.NodeCount())
	d.backwardVisitedNodes = make([]bool, d.g.NodeCount())
	d.origin = origin
	d.destination = destination
	if d.bidirectional {
		d.bidirectionalConnection = nil
	}
}

// Settle the given node item
func (d *UniversalDijkstra) settleNode(node *DijkstraItem) {
	if d.debugLevel >= 1 {
		fmt.Printf("Settling node %v, direction: %v\n", node.NodeId, node.searchDirection)
	}
	searchSpace, visitedNodes := d.searchSpace, d.visitedNodes
	if node.searchDirection == BACKWARD {
		searchSpace, visitedNodes = d.backwardSearchSpace, d.backwardVisitedNodes
	}
	searchSpace[node.NodeId] = node
	visitedNodes[node.NodeId] = true
}

// Check whether the given node item is settled in both search directions
func (d *UniversalDijkstra) isFullySettled(nodeId graph.NodeId) bool {
	visited := d.visitedNodes[nodeId]
	backwardVisited := d.backwardVisitedNodes[nodeId]
	return visited && backwardVisited
}

// Relax the Edges for the given node item and add the new nodes to the MinPath priority queue
func (d *UniversalDijkstra) relaxEdges(node *DijkstraItem, pq *MinPath) {
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
			if node.searchDirection == BACKWARD {
				// Default direction is forward
				// if it is backward, switch successor and predecessor
				connectionPredecessor, connectionSuccessor = connectionSuccessor, connectionPredecessor
			}
			con := NewBidirectionalConnection(connection.NodeId, connectionPredecessor, connectionSuccessor, node.distance+arc.Cost()+connection.distance)
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
			}
		}
	}
}

// Specify whether a heuristic for path finding (AStar) should be used
func (dijkstra *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	dijkstra.useHeuristic = useHeuristic
}

// Specify wheter the search should be done in both directions
func (dijkstra *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	dijkstra.bidirectional = bidirectional
}

// SPecify if the arc flags of the Arcs should be considered.
// If set to false, Arcs with a negative flag will be ignored by the path finding
func (d *UniversalDijkstra) SetConsiderArcFlags(considerArcFlags bool) {
	d.considerArcFlags = considerArcFlags
}

// Set the upper cost for a valid path from source to target
func (d *UniversalDijkstra) SetCostUpperBound(costUpperBound int) {
	d.costUpperBound = costUpperBound
}

// Set the maximum number of nodes that can get settled before the search is terminated
func (d *UniversalDijkstra) SetMaxNumSettledNodes(maxNumSettledNodes int) {
	d.maxNumSettledNodes = maxNumSettledNodes
}

// Set the debug level to show different debug messages.
// If it is 0, no debug messages are printed
func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}
