package path

import (
	"log"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type SearchStats struct {
	visitedNodes     []bool          // Array which indicates if a node (defined by index) was visited in the search
	searchSpace      []*DijkstraItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	stalledNodes     []bool          // indicates if the node (index=id) is stalled
	stallingDistance []int           // stalling distance for node (index=id)
}

type SearchKPIs struct {
	pqPops             int // store the amount of Pops which were performed on the priority queue for the computed search
	pqUpdates          int // store each update or push to the priority queue
	relaxationAttempts int // store the attempt for relaxed edges
	relaxedEdges       int // number of relaxed edges
	numSettledNodes    int // number of settled nodes
	stalledNodes       int // number of stalled nodes (invocations)
	unstalledNodes     int // number of unstalled nodes (invocations)
}

type SearchOptions struct {
	useHeuristic       bool   // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional      bool   // flag indicating if search should be done from both sides
	useHotStart        bool   // flag indicating if the previously cached results should get used
	considerArcFlags   bool   // flag indicating if arc flags (basically deactivate some edges) should be considered
	ignoreNodes        []bool // store for each node ID if it is ignored. A map would also be viable (for performance aspect) to achieve this
	stallOnDemand      int    // level for stall-on-demand. (0: no stalling, 1: regular stalling, 2: "preemptive" stalling, 3: stall recursively (regular), 4: stall recursive ("preemtpive"))
	sortedArcs         bool   // indicate if the arcs are sorted in the graph (according to enabled/disabled)
	costUpperBound     int    // upper bound of cost from origin to destination
	maxNumSettledNodes int    // maximum number of settled nodes before search is terminated
}

// UniversalDijkstra implements various path finding algorithms which all are based on Dijkstra.
// it can be used for plain Dijsktra, Bidirectional search, A* (and maybe more will come).
// Implements the Navigator Interface.
type UniversalDijkstra struct {
	g       graph.Graph
	minHeap queue.MinHeap[*DijkstraItem] // priority queue to find the shortest path

	origin      graph.NodeId // the origin of the current search
	destination graph.NodeId // the distination of the current search

	forwardSearch           SearchStats
	backwardSearch          SearchStats
	bidirectionalConnection BidirectionalConnection // contains the connection between the forward and backward search (if done bidirecitonal). If no connection is found, nodeId is -1

	searchOptions SearchOptions
	searchKPIs    SearchKPIs

	debugLevel int // debug level for logging purpose
}

// Describes the connection of a bidirectional search
type BidirectionalConnection struct {
	nodeId      graph.NodeId // the node id of the connecting node
	predecessor graph.NodeId // the node id of the predecessor
	successor   graph.NodeId // the node id of the successor
	distance    int          // the whole distance (in both directions)
}

// Reset the search stats
func (stats *SearchStats) Reset(size int, stallLevel int) {
	if stats.visitedNodes == nil {
		stats.visitedNodes = make([]bool, size)
	} else {
		for i := range stats.visitedNodes {
			stats.visitedNodes[i] = false
		}
	}
	if stats.searchSpace == nil {
		stats.searchSpace = make([]*DijkstraItem, size)
	} else {
		for i := range stats.searchSpace {
			stats.searchSpace[i] = nil
		}
	}

	if stallLevel > 0 {
		stats.stalledNodes = make([]bool, size)
		stats.stallingDistance = make([]int, size)
	}
}

// Reset the kpi
func (kpi *SearchKPIs) Reset() {
	kpi.pqPops = 0
	kpi.pqUpdates = 0
	kpi.relaxationAttempts = 0
	kpi.relaxedEdges = 0
	kpi.numSettledNodes = 0
	kpi.stalledNodes = 0
	kpi.unstalledNodes = 0
}

// Reset/invalidate the connection
func (con *BidirectionalConnection) Reset() {
	con.nodeId = -1
}

// Create a new Dijkstra instance with the given graph g
func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	options := SearchOptions{costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, ignoreNodes: make([]bool, g.NodeCount())}
	return &UniversalDijkstra{g: g, searchOptions: options, origin: -1, destination: -1}
}

// Compute the shortest path from the origin to the destination.
// It returns the length of the found path
// If no path was found, it returns -1
// If the path to all possible target is calculated (set target to -1), it returns 0
func (d *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
	if d.searchOptions.useHeuristic && d.searchOptions.bidirectional {
		panic("AStar doesn't work bidirectional so far.")
	}
	if d.searchOptions.bidirectional && destination < 0 {
		panic("Can't use bidirectional search with no specified destination.")
	}
	if d.searchOptions.bidirectional && d.searchOptions.useHotStart && d.searchOptions.considerArcFlags {
		panic("Can't use hot start with bidirecitonal search in directed graph")
	}
	if d.searchOptions.stallOnDemand > 0 && !d.searchOptions.considerArcFlags {
		panic("stall on demand works only on directed graph.")
	}

	if origin < 0 {
		panic("Origin invalid.")
	}

	if d.debugLevel >= 1 {
		log.Printf("New search: %v -> %v\n", origin, destination)
	}

	// helper function to adapt the connection when using hot start
	adaptConnection := func(connectionNodeId, predecessor, successor graph.NodeId, distance int) {
		d.bidirectionalConnection.nodeId = connectionNodeId
		d.bidirectionalConnection.distance = distance
		d.bidirectionalConnection.predecessor = predecessor
		d.bidirectionalConnection.successor = successor
	}

	if d.searchOptions.useHotStart {
		// These conditions don't work for bidirectional directed graphs (search), yet
		if d.origin == origin && destination > 0 && d.forwardSearch.visitedNodes[destination] {
			// hot start, already found the node in a previous search
			// just load the distance
			if d.debugLevel >= 1 {
				log.Printf("Using hot start, forward direction - loading %v -> %v, distance is %v\n", origin, destination, d.forwardSearch.searchSpace[destination].distance)
			}
			numSettledNodes := d.searchKPIs.numSettledNodes
			d.searchKPIs.Reset()
			d.searchKPIs.numSettledNodes = numSettledNodes

			adaptConnection(destination, d.forwardSearch.searchSpace[destination].predecessor, -1, d.forwardSearch.searchSpace[destination].distance)
			d.destination = destination
			return d.forwardSearch.searchSpace[destination].distance
		}
		if d.searchOptions.bidirectional && d.destination == destination && d.backwardSearch.visitedNodes[origin] {
			if d.debugLevel >= 1 {
				log.Printf("Using hot start, backward direction - loading %v -> %v, distance: %v\n", origin, destination, d.backwardSearch.searchSpace[origin].distance)
			}

			numSettledNodes := d.searchKPIs.numSettledNodes
			d.searchKPIs.Reset()
			d.searchKPIs.numSettledNodes = numSettledNodes

			adaptConnection(origin, -1, d.backwardSearch.searchSpace[origin].predecessor, d.backwardSearch.searchSpace[origin].distance)
			d.origin = origin
			return d.backwardSearch.searchSpace[origin].distance
		}
	}

	d.initializeSearch(origin, destination)

	for d.minHeap.Len() > 0 {
		currentNode := d.minHeap.Pop()
		d.searchKPIs.pqPops++
		if d.debugLevel >= 2 {
			log.Printf("Settling node %v, direction: %v, distance %v\n", currentNode.nodeId, currentNode.searchDirection, currentNode.distance)
		}

		d.settleNode(currentNode)

		stalledNodes, _ := alignWithSearchDirection(currentNode.searchDirection, d.forwardSearch.stalledNodes, d.backwardSearch.stalledNodes)

		if d.searchOptions.stallOnDemand >= 2 && stalledNodes[currentNode.nodeId] {
			// this is a stalled node -> nothing to do
			continue
		}

		d.relaxEdges(currentNode) // edges need to get relaxed before checking for termination to guarantee that hot start works. if peeking is used, this may get done a bit differently

		if currentNode.Priority() > d.searchOptions.costUpperBound || d.searchKPIs.numSettledNodes > d.searchOptions.maxNumSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			if d.debugLevel >= 1 {
				log.Printf("Exceeded limits - cost upper bound: %v, current cost: %v, max settled nodes: %v, current settled nodes: %v\n", d.searchOptions.costUpperBound, currentNode.Priority(), d.searchOptions.maxNumSettledNodes, d.searchKPIs.numSettledNodes)
			}
			return -1
		}

		if !d.searchOptions.considerArcFlags && d.bidirectionalConnection.nodeId != -1 && d.isFullySettled(currentNode.nodeId) {
			// check if the node was already settled in both directions, this is a connection
			// the connection item contains the information, which one is the real connection (this has not to be the current one)
			if d.debugLevel >= 1 {
				log.Printf("Finished search, distance: %v\n", d.bidirectionalConnection.distance)
			}
			return d.bidirectionalConnection.distance
		}

		if d.searchOptions.considerArcFlags && d.bidirectionalConnection.nodeId != -1 && d.bidirectionalConnection.distance < currentNode.Priority() {
			// exit condition for contraction hierarchies
			// if path is directed, it is not enough to test if node is settled from both sides, since the direction can block to reach the node from one side
			// for normal Dijkstra, this would force that the search space is in the bidirectional search as big as unidirecitonal search
			// check if the current visited node has already a higher priority (distance) than the connection. If this is the case, no lower connection can get found
			if d.debugLevel >= 1 {
				log.Printf("Finished search, distance: %v\n", d.bidirectionalConnection.distance)
			}
			return d.bidirectionalConnection.distance
		}

		if destination < 0 {
			// calculate path to everywhere, no need to check if destination is reached
			continue
		}

		if currentNode.searchDirection == FORWARD && currentNode.nodeId == destination {
			if d.debugLevel >= 1 {
				log.Printf("Found path %v -> %v with distance %v\n", origin, destination, d.forwardSearch.searchSpace[destination].distance)
			}
			return d.forwardSearch.searchSpace[destination].distance
		} else if currentNode.searchDirection == BACKWARD && currentNode.nodeId == origin {
			if d.bidirectionalConnection.nodeId == -1 {
				panic("connection should not be nil.")
			}
			if d.debugLevel >= 1 {
				log.Printf("Finished search, distance: %v\n", d.bidirectionalConnection.distance)
			}
			return d.bidirectionalConnection.distance
		}

	}

	if destination == -1 {
		if d.debugLevel >= 1 {
			log.Printf("Finished search, distance: 0\n")
		}
		// calculated every distance from source to each possible target
		return 0
	}

	if d.searchOptions.bidirectional {
		if d.bidirectionalConnection.nodeId == -1 {
			// no valid path found
			if d.debugLevel >= 1 {
				log.Printf("Finished search, no path found\n")
			}
			return -1
		}
		if d.debugLevel >= 1 {
			log.Printf("Finished search, distance: %v\n", d.bidirectionalConnection.distance)
		}
		return d.bidirectionalConnection.distance
	}

	if d.forwardSearch.searchSpace[destination] == nil {
		// no valid path found
		if d.debugLevel >= 1 {
			log.Printf("Finished search, no path found\n")
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
	if origin == destination {
		// origin and destination is the same -> path with one node is the result
		// this is a workaround, since for bidirectional search, the connection is not found (there exists no connection)
		return []int{origin}
	}

	path := make([]int, 0)

	if d.searchOptions.bidirectional {
		if d.bidirectionalConnection.nodeId == -1 {
			// no path found
			return make([]int, 0)
		}
		if d.debugLevel >= 1 {
			log.Printf("con: %v, pre: %v, suc: %v\n", d.bidirectionalConnection.nodeId, d.bidirectionalConnection.predecessor, d.bidirectionalConnection.successor)
		}
		for nodeId := d.bidirectionalConnection.predecessor; nodeId != -1; nodeId = d.forwardSearch.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseInPlace(path)
		path = append(path, d.bidirectionalConnection.nodeId)
		for nodeId := d.bidirectionalConnection.successor; nodeId != -1; nodeId = d.backwardSearch.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	} else {
		if d.forwardSearch.searchSpace[destination] == nil {
			// no path found
			return make([]int, 0)
		}
		for nodeId := destination; nodeId != -1; nodeId = d.forwardSearch.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		// reverse path (to create the correct direction)
		slice.ReverseInPlace(path)
	}

	return path
}

// Returns the search space of a previous computation. This contains all items which were settled.
func (d *UniversalDijkstra) GetSearchSpace() []*DijkstraItem {
	searchSpace := make([]*DijkstraItem, 0)
	for nodeId, visited := range d.forwardSearch.visitedNodes {
		if visited {
			searchSpace = append(searchSpace, d.forwardSearch.searchSpace[nodeId])
		}
	}
	for nodeId, visited := range d.backwardSearch.visitedNodes {
		if visited {
			searchSpace = append(searchSpace, d.backwardSearch.searchSpace[nodeId])
		}
	}
	return searchSpace
}

// Initialize a new search
// This resets the search space and visited nodes (and all other leftovers of a previous search)
func (d *UniversalDijkstra) initializeSearch(origin, destination graph.NodeId) {
	if d.debugLevel >= 99 {
		var visitedNodes []graph.NodeId
		for node, visited := range d.forwardSearch.visitedNodes {
			if visited {
				visitedNodes = append(visitedNodes, node)
			}
		}
		log.Printf("visited nodes: %v\n", visitedNodes)
	}

	if d.searchOptions.useHotStart {
		if d.origin == origin && d.destination == destination {
			// everything the same, nothing to do
			return
		}

		d.bidirectionalConnection.Reset()

		// helper function to restore the heap.
		// remove all items which don't match for the hot start (because of chenged origin/destination) and add a new initial item
		restoreHeap := func(source graph.NodeId, searchStats *SearchStats, heuristic int, searchDirection Direction) {
			// clean heap
			for _, item := range searchStats.searchSpace {
				// remove old items from the heap
				if item != nil && item.index > -1 {
					d.minHeap.Remove(item.index)
				}
			}
			searchStats.Reset(d.g.NodeCount(), d.searchOptions.stallOnDemand)
			targetItem := NewDijkstraItem(source, 0, -1, heuristic, searchDirection)
			d.minHeap.Push(targetItem)
			d.settleNode(targetItem)
		}

		if d.origin == origin {
			if d.debugLevel >= 1 {
				log.Printf("Use hot start, changed destination\n")
			}

			d.destination = destination

			if d.searchOptions.bidirectional {
				restoreHeap(d.destination, &d.backwardSearch, 0, BACKWARD)
			}

			return
		}

		if d.searchOptions.bidirectional && d.destination == destination {
			if d.debugLevel >= 1 {
				log.Printf("Use hot start, changed origin\n")
			}

			d.origin = origin

			restoreHeap(d.origin, &d.forwardSearch, 0, FORWARD)
			return
		}
	}

	// don't or can't use hot start
	if d.debugLevel >= 2 {
		log.Printf("Initialize new search, origin: %v\n", origin)
	}

	d.origin = origin
	d.destination = destination

	d.forwardSearch.Reset(d.g.NodeCount(), d.searchOptions.stallOnDemand)
	d.searchKPIs.Reset()
	d.bidirectionalConnection.Reset()

	// Initialize priority queue
	heuristic := heuristicValue(d.searchOptions.useHeuristic, d.g, d.origin, d.destination)
	originItem := NewDijkstraItem(d.origin, 0, -1, heuristic, FORWARD)
	d.minHeap = *queue.NewMinHeap[*DijkstraItem](nil)
	d.minHeap.Push(originItem)
	d.settleNode(originItem)

	// for bidirectional algorithm
	if d.searchOptions.bidirectional {
		d.backwardSearch.Reset(d.g.NodeCount(), d.searchOptions.stallOnDemand)

		destinationItem := NewDijkstraItem(d.destination, 0, -1, 0, BACKWARD)
		d.minHeap.Push(destinationItem)
		d.settleNode(destinationItem)
	}
}

// Settle the given node item
func (d *UniversalDijkstra) settleNode(node *DijkstraItem) {
	searchStat, _ := alignWithSearchDirection(node.searchDirection, &d.forwardSearch, &d.backwardSearch)
	searchSpace := searchStat.searchSpace
	visitedNodes := searchStat.visitedNodes
	d.searchKPIs.numSettledNodes++
	searchSpace[node.nodeId] = node
	visitedNodes[node.nodeId] = true
}

// Check whether the given node item is settled in both search directions
func (d *UniversalDijkstra) isFullySettled(nodeId graph.NodeId) bool {
	visited := d.forwardSearch.visitedNodes[nodeId]
	backwardVisited := d.backwardSearch.visitedNodes[nodeId]
	return visited && backwardVisited
}

// Relax the Edges for the given node item and add the new nodes to the MinPath priority queue
func (d *UniversalDijkstra) relaxEdges(node *DijkstraItem) {
	for _, arc := range d.g.GetArcsFrom(node.nodeId) {
		d.searchKPIs.relaxationAttempts++
		successor := arc.Destination()

		if d.searchOptions.ignoreNodes[successor] {
			// ignore this node
			if d.debugLevel >= 3 {
				log.Printf("Ignore Edge %v -> %v, because target is in ignore list\n", node.nodeId, successor)
			}
			continue
		}

		searchStats, inverseSearchStats := alignWithSearchDirection(node.searchDirection, &d.forwardSearch, &d.backwardSearch)
		searchSpace, inverseSearchSpace := searchStats.searchSpace, inverseSearchStats.searchSpace
		stalledNodes := searchStats.stalledNodes
		stallingDistance := searchStats.stallingDistance

		if d.searchOptions.considerArcFlags && !arc.ArcFlag() {
			// ignore this arc
			if d.debugLevel >= 3 {
				log.Printf("Ignore Edge %v -> %v\n", node.nodeId, successor)
			}

			if d.searchOptions.sortedArcs {
				// since edges are sorted, the following edges will also be disabled
				// however, this disables the check for preemptive stalling nodes
				break
			}

			// but first, check for stall-on-demand ("preemptive", inverse arc direction)
			if (d.searchOptions.stallOnDemand == 2 || d.searchOptions.stallOnDemand == 4) && searchSpace[successor] != nil && node.Priority()+arc.Cost() < searchSpace[successor].Priority() {
				if d.debugLevel >= 3 {
					log.Printf("Stall Node %v\n", successor)
				}
				d.stallNode(searchSpace[successor], node.distance+arc.Cost())
			}
			continue
		}

		// check for "regular" stall-on-demand
		if (d.searchOptions.stallOnDemand == 1 || d.searchOptions.stallOnDemand == 3) && searchSpace[successor] != nil && searchSpace[successor].Priority()+arc.Cost() < node.Priority() {
			d.stallNode(node, searchSpace[successor].distance+arc.Cost())
			continue
		}

		if d.debugLevel >= 3 {
			log.Printf("Relax Edge %v -> %v\n", node.nodeId, arc.Destination())
		}

		if d.searchOptions.bidirectional && inverseSearchSpace[successor] != nil {
			// store potential connection node, needed for later
			// this is a "real" copy, not just a pointer since it get changed now
			connection := inverseSearchSpace[successor]
			// Default direction is forward
			// if it is backward, switch successor and predecessor
			connectionPredecessor, connectionSuccessor := alignWithSearchDirection(node.searchDirection, node.nodeId, connection.predecessor)

			connectionDistance := node.distance + arc.Cost() + connection.distance
			if d.bidirectionalConnection.nodeId == -1 || connectionDistance < d.bidirectionalConnection.distance {
				d.bidirectionalConnection.nodeId = connection.nodeId
				d.bidirectionalConnection.distance = connectionDistance
				d.bidirectionalConnection.predecessor = connectionPredecessor
				d.bidirectionalConnection.successor = connectionSuccessor
			}
		}

		if searchSpace[successor] == nil {
			cost := node.distance + arc.Cost()
			heuristic := heuristicValue(d.searchOptions.useHeuristic, d.g, successor, d.destination)
			nextNode := NewDijkstraItem(successor, cost, node.nodeId, heuristic, node.searchDirection)
			searchSpace[successor] = nextNode
			d.minHeap.Push(nextNode)
			d.searchKPIs.pqUpdates++
		} else if updatedPriority := node.distance + arc.Cost() + searchSpace[successor].heuristic; updatedPriority < searchSpace[successor].Priority() {
			if d.searchOptions.stallOnDemand >= 1 && stalledNodes[successor] && node.distance+arc.Cost() <= stallingDistance[successor] {
				d.unstallNode(searchSpace[successor], node.distance+arc.Cost())
			} else {
				searchSpace[successor].distance = node.distance + arc.Cost()
				d.minHeap.Update(searchSpace[successor])
				d.searchKPIs.pqUpdates++
			}
			searchSpace[successor].predecessor = node.nodeId
		}
		d.searchKPIs.relaxedEdges++
	}
}

// Stall the given node with the stallingDistance
func (d *UniversalDijkstra) stallNode(node *DijkstraItem, stallingDistance int) {
	searchStat, _ := alignWithSearchDirection(node.searchDirection, &d.forwardSearch, &d.backwardSearch)
	searchSpace := searchStat.searchSpace
	stalledNodes := searchStat.stalledNodes
	stallingDistances := searchStat.stallingDistance

	// helper function to perform a breadth-first search for stalling additional nodes
	// the search stops at nodes which are already stalled or not going to be stalled or if maxHops is reached
	// this also sets the stalling distance for the stalled node
	bfs := func(nodeId graph.NodeId, initialDistance int, searchSpace []*DijkstraItem, stalledNodes []bool, stallingDistances []int, maxHops int) []graph.NodeId {
		queue := []graph.NodeId{nodeId}
		stallingDistances[nodeId] = initialDistance
		stallNodes := []graph.NodeId{nodeId}

		hops := 0
		for len(queue) > 0 {
			nodeId := queue[0]
			queue = queue[1:]
			hops++
			if hops >= maxHops {
				break
			}

			arcs := d.g.GetArcsFrom(nodeId)

			for i := range arcs {
				arc := &arcs[i]
				if !arc.ArcFlag() {
					// this arc is deactivated
					continue
				}

				destination := arc.Destination()
				cost := arc.Cost()
				stallingDistance := stallingDistances[nodeId] + cost

				if stalledNodes[destination] {
					// already stalled
					continue
				}
				if searchSpace[destination] == nil {
					// node not even considered in real search
					continue
				}
				if tentativeDistance := searchSpace[destination].distance; tentativeDistance <= stallingDistance {
					// this node won't be stalled, since its tentative distance is smaller than the stalling distance
					continue
				}

				if stallingDistances[destination] != 0 && stallingDistances[destination] <= stallingDistance {
					// already lower stalling distance available
					continue
				}

				// add node to queue
				stallingDistances[destination] = stallingDistance
				queue = append(queue, destination)
				stallNodes = append(stallNodes, nodeId)
			}

		}

		return stallNodes
	}

	var stallNodes []graph.NodeId
	if d.searchOptions.stallOnDemand <= 2 {
		// just stall the current node
		stallingDistances[node.NodeId()] = stallingDistance
		stallNodes = []graph.NodeId{node.NodeId()}
	} else if d.searchOptions.stallOnDemand >= 3 {
		// stall recursively
		// however, this takes very long time (even for very few hops)
		stallNodes = bfs(node.NodeId(), stallingDistance, searchSpace, stalledNodes, stallingDistances, math.MaxInt)
	}
	for _, nodeId := range stallNodes {
		stalledNodes[nodeId] = true
		d.searchKPIs.stalledNodes++

		if searchSpace[nodeId].index >= 0 {
			d.minHeap.Remove(searchSpace[nodeId].index)
		}
	}
}

// Unstall the given node and update the given distance
func (d *UniversalDijkstra) unstallNode(node *DijkstraItem, distance int) {
	searchStats, _ := alignWithSearchDirection(node.searchDirection, d.forwardSearch, d.backwardSearch)
	stalledNodes := searchStats.stalledNodes
	stallingDistance := searchStats.stallingDistance
	stalledNodes[node.nodeId] = false
	stallingDistance[node.nodeId] = 0
	d.searchKPIs.unstalledNodes++

	node.distance = distance
	if node.index < 0 {
		d.minHeap.Push(node)
	} else {
		d.minHeap.Update(node)
	}
}

// Specify whether a heuristic for path finding (AStar) should be used
func (d *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	d.searchOptions.useHeuristic = useHeuristic
}

// Specify wheter the search should be done in both directions
func (d *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	d.searchOptions.bidirectional = bidirectional
}

// SPecify if the arc flags of the Arcs should be considered.
// If set to false, Arcs with a negative flag will be ignored by the path finding
func (d *UniversalDijkstra) SetConsiderArcFlags(considerArcFlags bool) {
	d.searchOptions.considerArcFlags = considerArcFlags
}

// Set the upper cost for a valid path from source to target
func (d *UniversalDijkstra) SetCostUpperBound(costUpperBound int) {
	d.searchOptions.costUpperBound = costUpperBound
}

// Set the maximum number of nodes that can get settled before the search is terminated
func (d *UniversalDijkstra) SetMaxNumSettledNodes(maxNumSettledNodes int) {
	d.searchOptions.maxNumSettledNodes = maxNumSettledNodes
}

// Set the nodes which are ignored in the search
func (d *UniversalDijkstra) SetIgnoreNodes(nodes []graph.NodeId) {
	d.searchOptions.ignoreNodes = make([]bool, d.g.NodeCount())
	for _, node := range nodes {
		d.searchOptions.ignoreNodes[node] = true
	}
	// invalidate previously calculated results
	d.origin = -1
	d.destination = -1
}

// Use a hot start
func (d *UniversalDijkstra) SetHotStart(useHotStart bool) {
	d.searchOptions.useHotStart = useHotStart
}

// Use stall on demand
func (d *UniversalDijkstra) SetStallOnDemand(level int) {
	d.searchOptions.stallOnDemand = level
}

// use sorted arcs (for early termination)
func (d *UniversalDijkstra) SortedArcs(sorted bool) {
	d.searchOptions.sortedArcs = sorted
}

// Returns the amount of priority queue/heap pops which werer performed during the search
func (d *UniversalDijkstra) GetPqPops() int { return d.searchKPIs.pqPops }

// Get the number of relaxed edges
func (d *UniversalDijkstra) GetEdgeRelaxations() int { return d.searchKPIs.relaxedEdges }

// Get the number of attempted edge relaxations (some may early terminated)
func (d *UniversalDijkstra) GetRelaxationAttempts() int { return d.searchKPIs.relaxationAttempts }

// Get the number of pq updates
func (d *UniversalDijkstra) GetPqUpdates() int { return d.searchKPIs.pqUpdates }

// Get the number of stalling invocations
func (d *UniversalDijkstra) GetStalledNodesCount() int { return d.searchKPIs.stalledNodes }

// Get the number of unstall invocations
func (d *UniversalDijkstra) GetUnstalledNodesCount() int { return d.searchKPIs.unstalledNodes }

// Get the used graph
func (d *UniversalDijkstra) GetGraph() graph.Graph { return d.g }

// Set the debug level to show different debug messages.
// If it is 0, no debug messages are printed
func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}

// helper function to align the given items a,b with the search direction.
// if FORWARD, a and b don't change
// if BACKWARD, a and b are swapped
func alignWithSearchDirection[T any](searchDirection Direction, a, b T) (T, T) {
	if searchDirection == FORWARD {
		return a, b
	} else if searchDirection == BACKWARD {
		return b, a
	}
	panic("Search direction not supported")
}

// helper function for AStar to calculate the heuristic value from origin to destination
// Returns 0 if useHeuristic is false
func heuristicValue(useHeuristic bool, g graph.Graph, origin, destination graph.NodeId) int {
	if useHeuristic {
		return int(0.99 * float64(g.GetNode(origin).IntHaversine(g.GetNode(destination))))
	}
	return 0
}
