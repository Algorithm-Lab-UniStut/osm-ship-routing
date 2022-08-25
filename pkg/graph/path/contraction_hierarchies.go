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

type OrderOptions byte

const (
	dynamic OrderOptions = 1 << iota
	random
	considerEdgeDifference
	considerProcessedNeighbors
	periodic
)

func MakeOrderOptions() OrderOptions {
	return OrderOptions(0)
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

func (ch *ContractionHierarchies) SetNodeOrdering(nodeOrdering []int) {
	ch.nodeOrdering = nodeOrdering
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

func (ch *ContractionHierarchies) ComputeInitialNodeOrdering(givenNodeOrder []int, oo OrderOptions) *NodeOrder {
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
			ed, processedNeighbors := ch.ContractNode(i, true)
			orderItem := NewOrderItem(i)
			if oo.ConsiderEdgeDifference() {
				orderItem.edgeDifference = ed
			}
			if oo.ConsiderProcessedNeighbors() {
				orderItem.processedNeighbors = processedNeighbors
			}
			fmt.Printf("Add node %6v, edge difference: %3v, processed neighbors: %3v\n", i, ed, processedNeighbors)
			heap.Push(pq, orderItem)
		}
	}
	return pq
}

func (ch *ContractionHierarchies) ContractNode(nodeId graph.NodeId, computeEdgeDifferenceOnly bool) (int, int) {
	g, ok := ch.g.(graph.DynamicGraph)
	if !ok {
		panic("Adding edge not possible, Make sure to provide an dynamic graph")
	}
	if ch.debugLevel == 1 {
		fmt.Printf("Contract Node %v\n", nodeId)
	}
	shortcutsForNode := 0
	contractedNeighbors := 0
	arcs := ch.g.GetArcsFrom(nodeId)
	incidentArcs := len(arcs)
	if ch.debugLevel == 1 {
		fmt.Printf("Incident arcs %v\n", incidentArcs)
	}
	ch.disableArcsForNode(nodeId)
	//ch.dijkstra.SetConsiderArcFlags(true)
	for _, arc := range arcs {
		source := arc.Destination()
		if ch.isNodeAlreadyProcessed(source) {
			// if the position/order of the (current) nodeId is 0, no other node can be processed
			// if the other node has already an entry in the order, this was already processed
			// ch.orderOfNode[nodeId] != 0 && ch.orderOfNode[source] != 0 --> wrong, if source was processed at first
			if ch.debugLevel == 2 {
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
				if ch.debugLevel == 2 {
					fmt.Printf("target %v already processed\n", target)
				}
				//contractedNeighbors++
				continue
			}
			if ch.debugLevel == 1 {
				fmt.Printf("testing for shortcut %v -> %v\n", source, target)
			}
			maxCost := arc.Cost() + otherArc.Cost()
			ch.dijkstra.SetCostUpperBound(maxCost)
			length := ch.dijkstra.ComputeShortestPath(source, target)
			if ch.debugLevel == 1 {
				fmt.Printf("Length: %v, cost via node: %v\n", length, maxCost)
			}
			if length == -1 || length > maxCost {
				// add shortcut, since the path via this node is the fastest
				// without this node, the target is either not reachable or the path is longer
				shortcutsForNode++
				if !computeEdgeDifferenceOnly {
					shortcut := graph.NewEdge(target, source, maxCost)
					g.AddEdge(*shortcut)
					sc := Shortcut{source: source, target: target, via: nodeId}
					ch.shortcuts = append(ch.shortcuts, sc)
					// maybe this map is not so a good idea
					/*
						if ch.shortcutMap[source] == nil {
							ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
						}
						ch.shortcutMap[source][target] = nodeId
					*/
					if ch.debugLevel == 1 {
						fmt.Printf("Add shortcut %v %v %v %v\n", source, target, nodeId, maxCost)
					}
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

	if ch.debugLevel == 1 {
		fmt.Printf("Needed Shortcuts: %v, incident Arcs: %v, contractedNeighbors: %v\n", shortcutsForNode/2, incidentArcs, contractedNeighbors)
	}
	// number of shortcuts is doubled, since we have two arcs for each each (because symmetric graph)
	return shortcutsForNode/2 - incidentArcs, contractedNeighbors
}

func (ch *ContractionHierarchies) ContractNodes(order *NodeOrder, oo OrderOptions) {
	if order.Len() != ch.g.NodeCount() {
		// this is a rudimentary test, if the ordering could be valid.
		// However, it misses to test if every id appears exactly once
		panic("Node ordering not valid")
	}
	level := 0
	intermediateUpdates := 0
	for order.Len() > 0 {
		pqItem := heap.Pop(order).(*OrderItem)
		currentEdgeDifference, currentProcessedNeighbors := ch.ContractNode(pqItem.nodeId, true)
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
			ch.ContractNode(pqItem.nodeId, false)
			fmt.Printf("Level %6v - Contract Node %6v, heap length: %6v, edge difference: %3v, processed neighbors: %3v, updates: %6v\n", level, pqItem.nodeId, order.Len(), currentEdgeDifference, currentProcessedNeighbors, intermediateUpdates)
			intermediateUpdates = 0
			level++
		} else {
			if ch.debugLevel == 1 {
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
				orderItem := (*order)[i]
				ed, pd := ch.ContractNode(orderItem.nodeId, true)
				if !oo.ConsiderEdgeDifference() {
					ed = 0
				}
				if !oo.ConsiderProcessedNeighbors() {
					pd = 0
				}
				order.update(orderItem, ed, pd)
			}
			// not informative here?
			intermediateUpdates++
		}
	}
}

func (ch *ContractionHierarchies) Precompute(givenNodeOrder []int, oo OrderOptions) {
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(false)
	ch.dijkstra.SetUseHeuristic(false)
	// maybe test without maxNumSettledNodes
	ch.dijkstra.SetMaxNumSettledNodes(20)
	ch.addedShortcuts = make(map[int]int)
	//ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
	ch.shortcuts = make([]Shortcut, 0)
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	ch.nodeOrdering = make([]int, ch.g.NodeCount())
	if givenNodeOrder == nil && !oo.IsValid() {
		panic("Order Options are not valid")
	}
	for i := 0; i < ch.g.NodeCount(); i++ {
		ch.enableArcsForNode(i)
	}
	if givenNodeOrder != nil {
		// ignore the order options
		// -> just overwrite them
		oo = MakeOrderOptions().SetDynamic(false).SetRandom(false).SetEdgeDifference(false).SetProcessedNeighbors(false).SetRandom(false)
	}
	fmt.Printf("Compute Node Ordering\n")
	pq := ch.ComputeInitialNodeOrdering(givenNodeOrder, oo)
	fmt.Printf("Contract Nodes\n")
	ch.ContractNodes(pq, oo)
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

func (ch *ContractionHierarchies) IsShortcut(edge *graph.Edge) bool {
	// TODO is this necessary
	panic("not implemented")
}

func (ch *ContractionHierarchies) disableArcsForNode(nodeId graph.NodeId) {
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		// disable the outgoing arc
		if ch.debugLevel == 2 {
			fmt.Printf("disable arc %v -> %v\n", nodeId, arc.Destination())
		}
		arc.SetArcFlag(false)
	}
}

func (ch *ContractionHierarchies) enableArcsForNode(nodeId graph.NodeId) {
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		// enable the outgoing arc
		if ch.debugLevel == 2 {
			fmt.Printf("enable arc %v -> %v\n", nodeId, arc.Destination())
		}
		arc.SetArcFlag(true)
	}
}

func (ch *ContractionHierarchies) resetArcFlags() {
	for i := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(i) {
			arc.SetArcFlag(true)
		}
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

func (oo OrderOptions) Set(o OrderOptions) OrderOptions {
	return oo | o
}

func (oo OrderOptions) Reset(o OrderOptions) OrderOptions {
	return oo & ^o
}

func (oo OrderOptions) SetDynamic(flag bool) OrderOptions {
	if flag {
		return oo.Set(dynamic)
	} else {
		return oo.Reset(dynamic)
	}
}

func (oo OrderOptions) IsDynamic() bool {
	return oo&dynamic != 0
}

func (oo OrderOptions) SetRandom(flag bool) OrderOptions {
	if flag {
		return oo.Set(random)
	} else {
		return oo.Reset(random)
	}
}

func (oo OrderOptions) IsRandom() bool {
	return oo&random != 0
}

func (oo OrderOptions) SetEdgeDifference(flag bool) OrderOptions {
	if flag {
		return oo.Set(considerEdgeDifference)
	} else {
		return oo.Reset(considerEdgeDifference)
	}
}

func (oo OrderOptions) ConsiderEdgeDifference() bool {
	return oo&considerEdgeDifference != 0
}

func (oo OrderOptions) SetProcessedNeighbors(flag bool) OrderOptions {
	if flag {
		return oo.Set(considerProcessedNeighbors)
	} else {
		return oo.Reset(considerProcessedNeighbors)
	}
}

func (oo OrderOptions) ConsiderProcessedNeighbors() bool {
	return oo&considerProcessedNeighbors != 0
}

func (oo OrderOptions) SetPeriodic(flag bool) OrderOptions {
	if flag {
		return oo.Set(periodic)
	} else {
		return oo.Reset(periodic)
	}
}

func (oo OrderOptions) IsPeriodic() bool {
	return oo&periodic != 0
}

func (oo OrderOptions) IsValid() bool {
	if !oo.IsRandom() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using no random order, either the edge difference or the processed neighbors is needed for initial order computation
		return false
	}
	if oo.IsDynamic() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using dynamic order, either the edge difference or the processed neighbors (or both) must get considered
		return false
	}
	return true
}

func (ch *ContractionHierarchies) SetDebugLevel(level int) {
	ch.debugLevel = level
}
