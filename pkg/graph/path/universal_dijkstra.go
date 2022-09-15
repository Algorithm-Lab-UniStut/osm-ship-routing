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
	g  graph.Graph
	pq *MinPath // priority queue to find the shortest path

	origin      graph.NodeId // the origin of the current search
	destination graph.NodeId // the distination of the current search

	visitedNodes             []bool                  // Array which indicates if a node (defined by index) was visited in the forward search
	backwardVisitedNodes     []bool                  // Array which indicates if a node (defined by index) was visited in the backward search
	searchSpace              []*DijkstraItem         // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	backwardSearchSpace      []*DijkstraItem         // search space for the backward search
	bidirectionalConnection  BidirectionalConnection // contains the connection between the forward and backward search (if done bidirecitonal). If no connection is found, this is nil
	forwardStalledNodes      []bool                  //indicates if the node (index=id) is stalled
	backwardStalledNodes     []bool                  //indicates if the node (index=id) is stalled
	forwardStallingDistance  []int                   // stalling distance for node (index=id)
	backwardStallingDistance []int                   // stalling distance for node (index=id)

	useHeuristic       bool   // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional      bool   // flag indicating if search should be done from both sides
	useHotStart        bool   // flag indicating if the previously cached results should get used
	considerArcFlags   bool   // flag indicating if arc flags (basically deactivate some edges) should be considered
	ignoreNodes        []bool // store for each node ID if it is ignored. A map would also be viable (for performance aspect) to achieve this
	stallOnDemand      bool
	sortedArcs         bool
	costUpperBound     int // upper bound of cost from origin to destination
	maxNumSettledNodes int // maximum number of settled nodes before search is terminated

	pqPops          int // store the amount of Pops which were performed on the priority queue for the computed search
	pqUpdates       int // store each update or push to the priority queue
	edgeRelaxations int // store the attempt for relaxed edges
	numSettledNodes int // number of settled nodes
	debugLevel      int // debug level for logging purpose
}

type BidirectionalConnection struct {
	nodeId      graph.NodeId
	predecessor graph.NodeId
	successor   graph.NodeId
	distance    int
}

// Create a new Dijkstra instance with the given graph g
func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	return &UniversalDijkstra{g: g, costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, ignoreNodes: make([]bool, g.NodeCount()), origin: -1, destination: -1}
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
func (d *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
	if d.useHeuristic && d.bidirectional {
		panic("AStar doesn't work bidirectional so far.")
	}
	if d.useHotStart && d.bidirectional {
		panic("Can't use Hot start for bidirectional search.")
	}
	if d.bidirectional && destination < 0 {
		panic("Can't use bidirectional search with no specified destination.")
	}
	if d.useHotStart && destination < 0 {
		panic("Can't use Hot start when calculating path to everywhere.")
	}
	if d.stallOnDemand && !d.considerArcFlags {
		panic("stall on demand works only on directed graph.")
	}

	if origin < 0 {
		panic("Origin invalid.")
	}

	if d.useHotStart && d.origin == origin && d.visitedNodes[destination] {
		// hot start, already found the node in a previous search
		// just load the distance
		if d.debugLevel >= 1 {
			fmt.Printf("Using hot start - loading %v -> %v, distance is %v\n", origin, destination, d.searchSpace[destination].distance)
		}
		//d.destination = destination
		//d.bidirectionalConnection = nil
		d.pqPops = 0
		return d.searchSpace[destination].distance
	}

	d.initializeSearch(origin, destination)

	for d.pq.Len() > 0 {
		currentNode := heap.Pop(d.pq).(*DijkstraItem)
		d.pqPops++
		if d.debugLevel >= 2 {
			fmt.Printf("Settling node %v, direction: %v, distance %v\n", currentNode.NodeId, currentNode.searchDirection, currentNode.distance)
		}
		d.settleNode(currentNode)
		d.numSettledNodes++
		stalledNodes, backwardStalledNodes := d.forwardStalledNodes, d.backwardStalledNodes
		if currentNode.searchDirection == BACKWARD {
			stalledNodes, backwardStalledNodes = backwardStalledNodes, stalledNodes
		}
		if d.stallOnDemand && stalledNodes[currentNode.NodeId] {
			// this is a stalled node -> nothing to do
			continue
		}
		d.relaxEdges(currentNode) // edges need to get relaxed before checking for termination to guarantee that hot start works. if peeking is used, this may get done a bit differently
		if d.costUpperBound < currentNode.Priority() || d.maxNumSettledNodes < d.numSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			if d.debugLevel >= 2 {
				fmt.Printf("Exceeded limits - cost upper bound: %v, current cost: %v, max settled nodes: %v, current settled nodes: %v\n", d.costUpperBound, currentNode.Priority(), d.maxNumSettledNodes, d.numSettledNodes)
			}
			return -1
		}
		if !d.considerArcFlags && d.bidirectionalConnection.nodeId != -1 && d.isFullySettled(currentNode.NodeId) {
			// node with lowest priority is the current connection node
			// -> every edge increases cost/priority
			// -> this has to be the shortest path --> wrong, if one edge is (really) long
			// Correction: current node (with lowest possible distance) is already greater than connection distance
			// -> connection has to be the lowest one
			// -> Wrong, this condition just made a "normal" Dijkstra out of the Bidirectional Dijkstra
			// Correction: check if the node was already settled in both directions, this is a connection
			// the connection item contains the information, which one is the real connection (this has not to be the current one)
			if d.debugLevel >= 1 {
				fmt.Printf("Finished search\n")
			}
			return d.bidirectionalConnection.distance
		}
		if d.considerArcFlags && d.bidirectionalConnection.nodeId != -1 && currentNode.Priority() > d.bidirectionalConnection.distance {
			// exit condition for contraction hierarchies
			// if path is directed, it is not enough to test if node is settled from both sides, since the direction can block to reach the node from one side
			// for normal Dijkstra, this would force that the search space is in the bidirectional search as big as unidirecitonal search
			// check if the current visited node has already a higher priority (distance) than the connection. If this is the case, no lower connection can get found
			if d.debugLevel >= 1 {
				fmt.Printf("Finished search\n")
			}
			return d.bidirectionalConnection.distance
		}

		if destination != -1 {
			if currentNode.searchDirection == FORWARD && currentNode.NodeId == destination {
				if d.debugLevel >= 1 {
					fmt.Printf("Found path %v -> %v with distance %v\n", origin, destination, d.searchSpace[destination].distance)
				}
				return d.searchSpace[destination].distance
			} else if d.bidirectional && currentNode.searchDirection == BACKWARD && currentNode.NodeId == origin {
				// not necessary? - should be catched in bidirectionalConnection
				// appearently this can happen (at least for contraction hierarchies when calculated bidirectional)
				// -> first path/connection is found which has higher distance than possible (directed) path from destination to source
				if d.bidirectionalConnection.nodeId == -1 {
					// TODO Verify if this can happen
					panic("connection should not be nil")
				}
				if d.debugLevel >= 1 {
					fmt.Printf("Finished search\n")
				}
				return d.bidirectionalConnection.distance
			}
		}

	}

	if destination == -1 {
		if d.debugLevel >= 1 {
			fmt.Printf("Finished search\n")
		}
		// calculated every distance from source to each possible target
		return 0
	}

	if d.bidirectional {
		if d.debugLevel >= 1 {
			fmt.Printf("Finished search\n")
		}
		if d.bidirectionalConnection.nodeId == -1 {
			// no valid path found
			return -1
		}
		// TODO check why this can happen
		return d.bidirectionalConnection.distance
	}

	if d.searchSpace[destination] == nil {
		// no valid path found
		if d.debugLevel >= 1 {
			fmt.Printf("No path found\n")
		}
		return -1
	}

	panic("Normal search should get finished before")
}

// Get the path of a previous computation. This contains the nodeIds which lie on the path from source to destination
func (d *UniversalDijkstra) GetPath(origin, destination int) []int {
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0)
	}
	path := make([]int, 0)
	if d.bidirectional {
		if d.bidirectionalConnection.nodeId == -1 {
			// no path found
			return make([]int, 0)
		}
		if d.debugLevel >= 1 {
			fmt.Printf("con: %v, pre: %v, suc: %v\n", d.bidirectionalConnection.nodeId, d.bidirectionalConnection.predecessor, d.bidirectionalConnection.successor)
		}
		for nodeId := d.bidirectionalConnection.predecessor; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseIntInPlace(path)
		path = append(path, d.bidirectionalConnection.nodeId)
		for nodeId := d.bidirectionalConnection.successor; nodeId != -1; nodeId = d.backwardSearchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	} else {
		if d.searchSpace[destination] == nil {
			// no path found
			return make([]int, 0)
		}
		for nodeId := destination; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
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
	if d.debugLevel >= 99 {
		var visitedNodes []graph.NodeId
		for node, visited := range d.visitedNodes {
			if visited {
				visitedNodes = append(visitedNodes, node)
			}
		}
		fmt.Printf("visited nodes: %v\n", visitedNodes)
	}

	if !d.useHotStart || d.origin != origin {
		// don't or can't use hot start
		if d.debugLevel >= 2 {
			// TODO decide debug level
			fmt.Printf("Initialize new search, origin: %v\n", origin)
		}
		d.searchSpace = make([]*DijkstraItem, d.g.NodeCount())
		d.backwardSearchSpace = make([]*DijkstraItem, d.g.NodeCount())
		d.visitedNodes = make([]bool, d.g.NodeCount())
		d.backwardVisitedNodes = make([]bool, d.g.NodeCount())
		d.origin = origin
		d.destination = destination
		d.pqPops = 0
		d.pqUpdates = 0
		d.edgeRelaxations = 0
		d.numSettledNodes = 0
		d.bidirectionalConnection = BidirectionalConnection{nodeId: -1}

		// Initialize priority queue
		heuristic := 0
		if d.useHeuristic {
			heuristic = d.g.EstimateDistance(d.origin, d.destination)
		}
		originItem := NewDijkstraItem(d.origin, 0, -1, heuristic, FORWARD)
		d.pq = NewMinPath(originItem)
		d.settleNode(originItem)

		// for bidirectional algorithm
		if d.bidirectional {
			destinationItem := NewDijkstraItem(d.destination, 0, -1, 0, BACKWARD)
			heap.Push(d.pq, destinationItem)
			d.settleNode(destinationItem)
		}

		if d.stallOnDemand {
			d.forwardStalledNodes = make([]bool, d.g.NodeCount())
			d.backwardStalledNodes = make([]bool, d.g.NodeCount())
			d.forwardStallingDistance = make([]int, d.g.NodeCount())
			d.backwardStallingDistance = make([]int, d.g.NodeCount())
		}
	} else {
		if d.debugLevel >= 2 {
			// TODO decide debug level
			fmt.Printf("Use hot start\n")
		}
	}
}

// Settle the given node item
func (d *UniversalDijkstra) settleNode(node *DijkstraItem) {
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
func (d *UniversalDijkstra) relaxEdges(node *DijkstraItem) {
	for _, arc := range d.g.GetArcsFrom(node.NodeId) {
		d.edgeRelaxations++
		successor := arc.Destination()

		searchSpace, inverseSearchSpace := d.searchSpace, d.backwardSearchSpace
		stalledNodes, backwardStalledNodes := d.forwardStalledNodes, d.backwardStalledNodes
		stallingDistance, backwardStallingDistance := d.forwardStallingDistance, d.backwardStallingDistance

		if node.searchDirection == BACKWARD {
			searchSpace, inverseSearchSpace = inverseSearchSpace, searchSpace
			stalledNodes, backwardStalledNodes = backwardStalledNodes, stalledNodes
			stallingDistance, backwardStallingDistance = backwardStallingDistance, stallingDistance
		}

		if d.ignoreNodes[successor] {
			// ignore this node
			if d.debugLevel >= 3 {
				fmt.Printf("Ignore Edge %v -> %v, because target is in ignore list\n", node.NodeId, successor)
			}
			continue
		}

		if d.considerArcFlags && !arc.ArcFlag() {
			// ignore this arc
			if d.debugLevel >= 3 {
				fmt.Printf("Ignore Edge %v -> %v\n", node.NodeId, successor)
			}
			if d.sortedArcs {
				// since edges are sorted, the folowing edges will also be disabled
				// this disables the check for stalling nodes (in inverse direction)
				break
			}

			// but first, check for stall-on-demand
			if d.stallOnDemand && searchSpace[successor] != nil && node.Priority()+arc.Cost() < searchSpace[successor].Priority() {
				if d.debugLevel >= 3 {
					fmt.Printf("Stall Node %v\n", successor)
				}
				stalledNodes[successor] = true
				stallingDistance[successor] = node.Priority() + arc.Cost()

				// stall recursively
				// however, this takes very long time (even for only 1 level)
				// even just creating the Dijkstra object takes very long time
				/*
					dijkstra := NewUniversalDijkstra(d.g)
					dijkstra.SetConsiderArcFlags(true)
					dijkstra.SetMaxNumSettledNodes(1) // maybe better: specify depth
						dijkstra.ComputeShortestPath(successor, -1)
							for _, v := range dijkstra.GetSearchSpace() {
								// this check could be wrong
								// every node which is reachable in BFS should be stalled
								// except that ones which have shorter distance in dijkstra
								// -> if they are not in search space, they still can get stalled?
								if v != nil && v.Priority() > 0 && searchSpace[v.NodeId] != nil && node.Priority()+arc.Cost()+v.Priority() < searchSpace[v.NodeId].Priority() {
									stalledNodes[v.NodeId] = true
									stallingDistance[v.NodeId] = node.Priority() + arc.Cost() + v.Priority()
								}
							}
				*/
			}
			continue
		}

		if d.stallOnDemand && searchSpace[successor] != nil && searchSpace[successor].Priority()+arc.Cost() < node.Priority() {
			stalledNodes[node.NodeId] = true
			stallingDistance[node.NodeId] = searchSpace[successor].Priority() + arc.Cost()
			continue
		}

		if d.debugLevel >= 3 {
			fmt.Printf("Relax Edge %v -> %v\n", node.NodeId, arc.Destination())
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

			connectionDistance := node.distance + arc.Cost() + connection.distance
			if d.bidirectionalConnection.nodeId == -1 || connectionDistance < d.bidirectionalConnection.distance {
				d.bidirectionalConnection.nodeId = connection.NodeId
				d.bidirectionalConnection.distance = connectionDistance
				d.bidirectionalConnection.predecessor = connectionPredecessor
				d.bidirectionalConnection.successor = connectionSuccessor
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
			if d.stallOnDemand && nextNode.Priority() <= stallingDistance[successor] {
				stalledNodes[successor] = false
			}
			heap.Push(d.pq, nextNode)
			d.pqUpdates++
		} else {
			if updatedPriority := node.distance + arc.Cost() + searchSpace[successor].heuristic; updatedPriority < searchSpace[successor].Priority() {
				// TODO does this update work when stalled? or should this throw an error/push wrong node (because index=-1)
				d.pq.update(searchSpace[successor], node.distance+arc.Cost())
				d.pqUpdates++
				searchSpace[successor].predecessor = node.NodeId
				if d.stallOnDemand && updatedPriority <= stallingDistance[successor] {
					stalledNodes[successor] = false
					// need to push successor (again?) to queue?
					// was it removed before?
					heap.Push(d.pq, searchSpace[successor])
					d.pqUpdates++
				}
			}
		}
	}
}

// Specify whether a heuristic for path finding (AStar) should be used
func (d *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	d.useHeuristic = useHeuristic
}

// Specify wheter the search should be done in both directions
func (d *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	d.bidirectional = bidirectional
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

func (d *UniversalDijkstra) SetIgnoreNodes(nodes []graph.NodeId) {
	d.ignoreNodes = make([]bool, d.g.NodeCount())
	for _, node := range nodes {
		d.ignoreNodes[node] = true
	}
	// invalidate previously calculated results
	d.origin = -1
}

func (d *UniversalDijkstra) SetHotStart(useHotStart bool) {
	d.useHotStart = useHotStart
}

func (d *UniversalDijkstra) SetStallOnDemand(stallOnDemand bool) {
	d.stallOnDemand = stallOnDemand
}

func (d *UniversalDijkstra) SortedArcs(sorted bool) {
	d.sortedArcs = sorted
}

// Returns the amount of priority queue/heap pops which werer performed during the search
func (d *UniversalDijkstra) GetPqPops() int          { return d.pqPops }
func (d *UniversalDijkstra) GetEdgeRelaxations() int { return d.edgeRelaxations }
func (d *UniversalDijkstra) GetPqUpdates() int       { return d.pqUpdates }

// Set the debug level to show different debug messages.
// If it is 0, no debug messages are printed
func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}
