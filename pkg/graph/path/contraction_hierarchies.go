package path

import (
	"bufio"
	"container/heap"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strings"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type ContractionHierarchies struct {
	g            graph.Graph        // graph to work on (for precomputation, this ha sto be a graph.DynamicGraph)
	dijkstra     *UniversalDijkstra // the dijkstra algorithm to perform the searches
	nodeOrdering []graph.NodeId     // the node ordering (in which order the nodes were contracted)
	orderOfNode  []int              // the order of the node ("reverse" node ordering). At which position the specified node was contracted

	// decide for one
	//shortcutMap map[graph.NodeId]map[graph.NodeId]graph.NodeId // a map of the shortcuts (from/source -> to/target -> via)
	shortcuts []Shortcut // array which contains all shortcuts

	addedShortcuts       map[int]int // debug information - stores the number of how many nodes introduced the specified amount of shortcuts. Key is the number of shortcuts, value is how many introduced them
	debugLevel           int         // the debug level - used for printing some informaiton
	graphFilename        string      // the filename were the file gets stored
	shortcutsFilename    string      // the filename were the shourtcuts gets stored
	nodeOrderingFilename string      // the filname were the node ordering gets stored
}

type Shortcut struct {
	source graph.NodeId
	target graph.NodeId
	via    graph.NodeId
}

func NewContractionHierarchies(g graph.Graph, dijkstra *UniversalDijkstra) *ContractionHierarchies {
	return &ContractionHierarchies{g: g, dijkstra: dijkstra, graphFilename: "contracted_graph.fmi", shortcutsFilename: "shortcuts.txt", nodeOrderingFilename: "node_ordering.txt"}
}

func NewContractionHierarchiesInitialized(g graph.Graph, dijkstra *UniversalDijkstra, shortcuts []Shortcut, nodeOrdering []int) *ContractionHierarchies {
	ch := NewContractionHierarchies(g, dijkstra)
	ch.SetShortcuts(shortcuts)
	ch.SetNodeOrdering(nodeOrdering)
	return ch
}

func (ch *ContractionHierarchies) Precompute(givenNodeOrder []int, oo OrderOptions) {
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(false)
	ch.dijkstra.SetUseHeuristic(false)
	// maybe test without maxNumSettledNodes
	ch.dijkstra.SetMaxNumSettledNodes(100)
	ch.addedShortcuts = make(map[int]int)
	//ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
	ch.shortcuts = make([]Shortcut, 0)
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	ch.nodeOrdering = make([]int, ch.g.NodeCount())
	if givenNodeOrder == nil && !oo.IsValid() {
		panic("Order Options are not valid")
	}
	ch.g.EnableAllArcs()
	if givenNodeOrder != nil {
		// ignore the order options
		// -> just overwrite them
		oo = MakeOrderOptions().SetDynamic(false).SetRandom(false).SetEdgeDifference(false).SetProcessedNeighbors(false).SetRandom(false)
	}
	if ch.debugLevel >= 1 {
		fmt.Printf("Compute Node Ordering\n")
	}
	pq := ch.computeInitialNodeOrdering(givenNodeOrder, oo)
	if ch.debugLevel >= 1 {
		fmt.Printf("%v\n", pq)
		fmt.Printf("Contract Nodes\n")
	}
	ch.contractNodes(pq, oo)
}

func (ch *ContractionHierarchies) ComputeShortestPath(origin, destination graph.NodeId) int {
	ch.dijkstra.SetMaxNumSettledNodes(math.MaxInt)
	ch.dijkstra.SetCostUpperBound(math.MaxInt)
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(true)
	ch.dijkstra.SetUseHeuristic(false)
	ch.disableArcsAccordingToNodeOrder()
	length := ch.dijkstra.ComputeShortestPath(origin, destination)
	return length
}

func (ch *ContractionHierarchies) GetPath(origin, destination graph.NodeId) []int {
	// TODO: this probably gets more efficient with other data structures (store shortcuts as map -> faster access)
	path := ch.dijkstra.GetPath(origin, destination)
	if ch.debugLevel == 1 {
		fmt.Println(path)
	}
	for i := 0; i < len(path)-1; i++ {
		source := path[i]
		target := path[i+1]
		/*
			via, exists := ch.shortcutMap[source][target]
			if exists {
				path = slice.InsertInt(path, i+1, via)
				if ch.debugLevel == 1 {
					fmt.Printf("Added node %v -> %v -> %v\n", source, via, target)
				}
				i-- // reevaluate, if the source has a shortcut to the currently added node
			}
		*/
		for _, sc := range ch.shortcuts {
			if sc.source == source && sc.target == target {
				path = slice.InsertInt(path, i+1, sc.via)
				if ch.debugLevel == 1 {
					fmt.Printf("Added node %v -> %v -> %v\n", source, sc.via, target)
				}
				i-- // reevaluate, if the source has a shortcut to the currently added node
			}
		}

	}
	return path
}

func (ch *ContractionHierarchies) GetSearchSpace() []*DijkstraItem {
	return ch.dijkstra.GetSearchSpace()
}

func (ch *ContractionHierarchies) computeInitialNodeOrdering(givenNodeOrder []int, oo OrderOptions) *NodeOrder {
	var pq *NodeOrder
	if givenNodeOrder != nil {
		order := make(NodeOrder, ch.g.NodeCount())
		for i := 0; i < ch.g.NodeCount(); i++ {
			order[i] = NewOrderItem(givenNodeOrder[i])
			order[i].edgeDifference = i
		}
		pq = &order
		heap.Init(pq)
	} else if oo.IsRandom() {
		nodeOrdering := make([]int, ch.g.NodeCount())
		for i := range nodeOrdering {
			nodeOrdering[i] = i
		}
		rand.Seed(time.Now().UnixNano()) // completely random
		rand.Shuffle(len(nodeOrdering), func(i, j int) { nodeOrdering[i], nodeOrdering[j] = nodeOrdering[j], nodeOrdering[i] })

		order := make(NodeOrder, ch.g.NodeCount())
		for i := 0; i < ch.g.NodeCount(); i++ {
			order[i] = NewOrderItem(nodeOrdering[i])
			//order[i].edgeDifference = i
		}
		pq = &order
		heap.Init(pq)
	} else {
		pq = NewNodeOrder(nil)
		for i := 0; i < ch.g.NodeCount(); i++ {
			shortcuts, incidentArcs, processedNeighbors := ch.contractNode(i, true)
			orderItem := NewOrderItem(i)
			if oo.ConsiderEdgeDifference() {
				orderItem.edgeDifference = shortcuts - incidentArcs
			}
			if oo.ConsiderProcessedNeighbors() {
				orderItem.processedNeighbors = processedNeighbors
			}
			if ch.debugLevel >= 2 {
				fmt.Printf("Add node %6v, edge difference: %3v, processed neighbors: %3v\n", i, shortcuts-incidentArcs, processedNeighbors)
			}
			heap.Push(pq, orderItem)
		}
	}
	return pq
}

func (ch *ContractionHierarchies) contractNodes(order *NodeOrder, oo OrderOptions) {
	if order.Len() != ch.g.NodeCount() {
		// this is a rudimentary test, if the ordering could be valid.
		// However, it misses to test if every id appears exactly once
		panic("Node ordering not valid")
	}
	level := 0
	intermediateUpdates := 0
	for order.Len() > 0 {
		pqItem := heap.Pop(order).(*OrderItem)
		addedShortcuts, edges, currentProcessedNeighbors := ch.contractNode(pqItem.nodeId, true)
		currentEdgeDifference := addedShortcuts - edges
		if !oo.ConsiderEdgeDifference() {
			currentEdgeDifference = 0
		}
		if !oo.ConsiderProcessedNeighbors() {
			currentProcessedNeighbors = 0
		}
		if !oo.IsDynamic() || order.Len() == 0 || currentEdgeDifference+currentProcessedNeighbors <= order.Peek().(*OrderItem).Priority() {
			// always stick to initially computed order or this is still the smallest edge difference
			//fmt.Printf("Pos %6v: Contract Node %6v, heap length: %6v, intermediate updates: %6v\n", position, pqItem.nodeId, initialOrder.Len(), intermediateUpdates)
			if /*position > 0 && */ ch.orderOfNode[pqItem.nodeId] > 0 {
				panic("Node was already ordered?")
			}
			ch.orderOfNode[pqItem.nodeId] = level
			ch.nodeOrdering[level] = pqItem.nodeId
			ch.contractNode(pqItem.nodeId, false)
			if ch.debugLevel >= 1 {
				fmt.Printf("Level %6v - Contract Node %6v, heap: %6v, sc: %3v, edges: %3v, ed: %3v pn: %3v, prio: %3v, updates: %6v\n", level, pqItem.nodeId, order.Len(), addedShortcuts, edges, currentEdgeDifference, currentProcessedNeighbors, currentEdgeDifference+currentProcessedNeighbors, intermediateUpdates)
			}
			intermediateUpdates = 0
			level++
		} else {
			if ch.debugLevel == 2 {
				fmt.Printf("Update order\n")
			}
			if oo.ConsiderEdgeDifference() {
				pqItem.edgeDifference = currentEdgeDifference
			}
			if oo.ConsiderProcessedNeighbors() {
				pqItem.processedNeighbors = currentProcessedNeighbors
			}
			heap.Push(order, pqItem)
			intermediateUpdates++
		}
		if oo.IsPeriodic() && level%100 == 0 {
			for i := 0; i < order.Len(); i++ {
				orderItem := order.PeekAt(i).(*OrderItem)
				sc, edges, pn := ch.contractNode(orderItem.nodeId, true)
				ed := sc - edges
				if !oo.ConsiderEdgeDifference() {
					ed = 0
				}
				if !oo.ConsiderProcessedNeighbors() {
					pn = 0
				}
				order.update(orderItem, ed, pn)
			}
			// not informative here?
			intermediateUpdates++
		}
	}
}

func (ch *ContractionHierarchies) contractNode(nodeId graph.NodeId, computeEdgeDifferenceOnly bool) (int, int, int) {
	if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Contract Node %v\n", nodeId)
	}
	shortcutsForNode := 0
	contractedNeighbors := 0
	arcs := ch.g.GetArcsFrom(nodeId)
	incidentArcs := len(arcs)
	if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Incident arcs %v\n", incidentArcs)
	}
	ch.disableArcsForNode(nodeId)
	runtime := 0
	computations := 0
	for _, arc := range arcs {
		source := arc.Destination()
		if ch.isNodeAlreadyProcessed(source) {
			// if the position/order of the (current) nodeId is 0, no other node can be processed
			// if the other node has already an entry in the order, this was already processed
			// ch.orderOfNode[nodeId] != 0 && ch.orderOfNode[source] != 0 --> wrong, if source was processed at first
			if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("source %v already processed\n", source)
			}
			contractedNeighbors++
			continue
		}
		for _, otherArc := range ch.g.GetArcsFrom(nodeId) {
			target := otherArc.Destination()
			if source == target {
				// no need to process this path
				continue
			}
			if ch.isNodeAlreadyProcessed(target) {
				// if the position/order of the (current) nodeId is 0, no other node can be processed
				// if the other node has already an entry in the order, this was already processed
				// ch.orderOfNode[nodeId] != 0 && ch.orderOfNode[target] != 0 --> wrong, if target was processed at first
				if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
					fmt.Printf("target %v already processed\n", target)
				}
				//contractedNeighbors++
				continue
			}
			if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("testing for shortcut %v -> %v\n", source, target)
			}
			maxCost := arc.Cost() + otherArc.Cost()
			ch.dijkstra.SetCostUpperBound(maxCost)
			start := time.Now()
			length := ch.dijkstra.ComputeShortestPath(source, target)
			elapsed := time.Since(start)
			runtime += int(elapsed)
			computations++

			if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("Length: %v, cost via node: %v\n", length, maxCost)
			}
			if length == -1 || length > maxCost {
				// add shortcut, since the path via this node is the fastest
				// without this node, the target is either not reachable or the path is longer
				shortcutsForNode++
				if !computeEdgeDifferenceOnly {
					ch.addShortcut(source, target, nodeId, maxCost)
				}
			}

		}
	}
	if computeEdgeDifferenceOnly {
		// reset temporarily disabled arcs
		ch.enableArcsForNode(nodeId)
	} else {
		_, exists := ch.addedShortcuts[shortcutsForNode]
		if !exists {
			ch.addedShortcuts[shortcutsForNode] = 1
		} else {
			ch.addedShortcuts[shortcutsForNode]++
		}
	}

	if ch.debugLevel >= 1 && computations > 0 {
		fmt.Printf("Dijkstra Runtime: %v ms\n", float64(runtime/computations)/1000)
	}

	// Fix incident arcs: Remove number of already contracted neighbors
	incidentArcs -= contractedNeighbors

	if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Added Shortcuts: %v, incident Arcs: %v, contractedNeighbors: %v\n", shortcutsForNode/2, incidentArcs, contractedNeighbors)
	}
	// number of shortcuts is doubled, since we have two arcs for each each (because symmetric graph)
	return shortcutsForNode, incidentArcs, contractedNeighbors
}

func (ch *ContractionHierarchies) addShortcut(source, target, via graph.NodeId, cost int) {
	// TODO maybe convert this one time at the start to improve performance
	g, ok := ch.g.(graph.DynamicGraph)
	if !ok {
		panic("Adding edge not possible, Make sure to provide an dynamic graph")
	}
	shortcut := graph.NewEdge(target, source, cost)
	g.AddEdge(*shortcut)
	sc := Shortcut{source: source, target: target, via: via}
	ch.shortcuts = append(ch.shortcuts, sc)
	// maybe this map is not so a good idea
	/*
		if ch.shortcutMap[source] == nil {
			ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
		}
		ch.shortcutMap[source][target] = nodeId
	*/
	if ch.debugLevel == 2 {
		fmt.Printf("Add shortcut %v %v %v %v\n", source, target, via, cost)
	}
}

func (ch *ContractionHierarchies) isShortcut(edge *graph.Edge) bool {
	// TODO is this necessary
	panic("not implemented")
}

func (ch *ContractionHierarchies) enableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, true)
	if ch.debugLevel == 2 {
		fmt.Printf("enable arcs of node %v\n", nodeId)
	}
}

func (ch *ContractionHierarchies) disableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, false)
	if ch.debugLevel == 2 {
		fmt.Printf("disable arcs of node %v\n", nodeId)
	}
}

func (ch *ContractionHierarchies) disableArcsAccordingToNodeOrder() {
	for source := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(source) {
			target := arc.Destination()
			arc.SetArcFlag(ch.orderOfNode[source] < ch.orderOfNode[target])
		}
	}
}

func (ch *ContractionHierarchies) isNodeAlreadyProcessed(nodeId graph.NodeId) bool {
	// check whether the arcs of the node are already disabled
	// TODO: other options: check if the nodeId is on a lower position in the nodeOrdering array than the node which gets contracted
	arcs := ch.g.GetArcsFrom(nodeId)
	if len(arcs) == 0 || !arcs[0].ArcFlag() {
		// this node has no arcs
		// or the arc flag of (at least) one arc is already set to false
		return true
		// this check should be faster than iterating over all arcs
		// if one arc is modified, the whole node was processed
	}
	return false
	/*
		for _, arc := range ch.g.GetArcsFrom(nodeId) {
			if !arc.ArcFlag() {
				return true
			}
		}
		return false
	*/
}

func (ch *ContractionHierarchies) SetShortcuts(shortcuts []Shortcut) {
	ch.shortcuts = shortcuts
	/*
		ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
		for _, v := range shortcuts {
			source := v.source
			target := v.target
			via := v.via
			if ch.shortcutMap[source] == nil {
				ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
			}
			ch.shortcutMap[source][target] = via
		}
	*/
}

func (ch *ContractionHierarchies) GetShortcuts() []Shortcut {
	return ch.shortcuts
	/*
		sc := make([]Shortcut, 0)
		for source, shortcuts := range ch.shortcutMap {
			for target, via := range shortcuts {
				sc = append(sc, Shortcut{source: source, target: target, via: via})
			}
		}
		return sc
	*/
}

func (ch *ContractionHierarchies) SetNodeOrdering(nodeOrdering []int) {
	ch.nodeOrdering = nodeOrdering
	ch.orderOfNode = make([]int, len(nodeOrdering))
	for i, nodeId := range nodeOrdering {
		ch.orderOfNode[nodeId] = i
	}
}

func (ch *ContractionHierarchies) SetDebugLevel(level int) {
	ch.debugLevel = level
}

func (ch *ContractionHierarchies) WriteGraph() {
	graph.WriteFmi(ch.g, ch.graphFilename)
}

func (ch *ContractionHierarchies) WriteShortcuts() {
	file, cErr := os.Create(ch.shortcutsFilename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	var sb strings.Builder

	for _, v := range ch.GetShortcuts() {
		shortcut := fmt.Sprintf("%v %v %v\n", v.source, v.target, v.via)
		sb.WriteString(shortcut)
	}

	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

func (ch *ContractionHierarchies) WriteNodeOrdering() {
	file, cErr := os.Create(ch.nodeOrderingFilename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	var sb strings.Builder
	for _, v := range ch.nodeOrdering {
		order := fmt.Sprintf("%v\n", v)
		sb.WriteString(order)
	}
	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

func (ch *ContractionHierarchies) WriteContractionResult() {
	ch.WriteGraph()
	ch.WriteShortcuts()
	ch.WriteNodeOrdering()
}

func ReadShortcutFile(filename string) []Shortcut {
	file, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return ConvertToShortcuts(string(file))
}

func ConvertToShortcuts(shortcutsString string) []Shortcut {
	scanner := bufio.NewScanner(strings.NewReader(shortcutsString))

	shortcuts := make([]Shortcut, 0)
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 || line[0] == '#' {
			// skip empty lines and comments
			continue
		}
		var source, target, via graph.NodeId
		fmt.Scanf(line, "%d %d %d", source, target, via)
		shortcuts = append(shortcuts, Shortcut{source: source, target: target, via: via})
	}
	return shortcuts
}

func ReadNodeOrderingFile(filename string) []int {
	file, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return ConvertToNodeOrdering(string(file))
}

func ConvertToNodeOrdering(nodeOrderingString string) []int {
	scanner := bufio.NewScanner(strings.NewReader(nodeOrderingString))

	nodeOrdering := make([]int, 0)
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 || line[0] == '#' {
			// skip empty lines and comments
			continue
		}
		var nodeId graph.NodeId
		fmt.Scanf(line, "%d", nodeId)
		nodeOrdering = append(nodeOrdering, nodeId)
	}
	return nodeOrdering
}
