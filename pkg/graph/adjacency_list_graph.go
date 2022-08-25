package graph

import (
	"fmt"
	"strings"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
)

// TODO: Maybe add a graph class for "planar" geometry (not spherical)

// Implementation for dynamic graphs
type AdjacencyListGraph struct {
	Nodes     []Node
	Edges     [][]*Arc
	edgeCount int
}

func (alg *AdjacencyListGraph) GetNode(id NodeId) Node {
	if id < 0 || id >= alg.NodeCount() {
		panic(id)
	}
	return alg.Nodes[id]
}

func (alg *AdjacencyListGraph) GetNodes() []Node {
	return alg.Nodes
}

func (alg *AdjacencyListGraph) GetArcsFrom(id NodeId) []*Arc {
	if id < 0 || id >= alg.NodeCount() {
		panic(id)
	}
	arcs := make([]*Arc, 0)
	for _, arc := range alg.Edges[id] {
		arcs = append(arcs, arc)
	}
	return arcs
}

func (alg *AdjacencyListGraph) NodeCount() int {
	return len(alg.Nodes)
}

/*
func (alg *AdjacencyListGraph) EdgeCount() int {
	return alg.edgeCount
}
*/

func (alg *AdjacencyListGraph) ArcCount() int {
	return alg.edgeCount
}

func (alg *AdjacencyListGraph) AsString() string {
	var sb strings.Builder

	// write number of nodes and number of edges
	sb.WriteString(fmt.Sprintf("%v\n", alg.NodeCount()))
	sb.WriteString(fmt.Sprintf("%v\n", alg.ArcCount()))

	sb.WriteString(fmt.Sprintf("#Nodes\n"))
	// list all nodes structured as "id lat lon"
	for i := 0; i < alg.NodeCount(); i++ {
		node := alg.GetNode(i)
		sb.WriteString(fmt.Sprintf("%v %v %v\n", i, node.Lat, node.Lon))
	}

	sb.WriteString(fmt.Sprintf("#Edges\n"))
	// list all edges structured as "fromId targetId distance"
	for i := 0; i < alg.NodeCount(); i++ {
		for _, arc := range alg.GetArcsFrom(i) {
			sb.WriteString(fmt.Sprintf("%v %v %v\n", i, arc.Destination(), arc.Cost()))
		}
	}
	return sb.String()
}

func (alg *AdjacencyListGraph) AddNode(n Node) {
	alg.Nodes = append(alg.Nodes, n)
	alg.Edges = append(alg.Edges, make([]*Arc, 0))
}

func (alg *AdjacencyListGraph) AddEdge(e Edge) {
	// Check if both source and target node exit
	if e.From >= alg.NodeCount() || e.To >= alg.NodeCount() {
		panic(fmt.Sprintf("Edge out of range %v", e))
	}
	// Check for duplicates
	for _, arc := range alg.Edges[e.From] {
		if e.To == arc.To {
			return // ignore duplicate edges
		}
	}
	alg.Edges[e.From] = append(alg.Edges[e.From], e.toArc())
	alg.edgeCount++
}

func (alg *AdjacencyListGraph) AddArc(from, to NodeId, distance int) {
	if from >= alg.NodeCount() || to >= alg.NodeCount() {
		panic(fmt.Sprintf("Arc out of range %v -> %v", from, to))
	}
	// Check for duplicates
	for _, arc := range alg.Edges[from] {
		if to == arc.To {
			return // ignore duplicate edges
		}
	}
	alg.Edges[from] = append(alg.Edges[from], NewArc(to, distance, true))
	alg.edgeCount++
}

func (alg *AdjacencyListGraph) EstimateDistance(source, target NodeId) int {
	origin := alg.GetNode(source)
	destination := alg.GetNode(target)
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return originPoint.IntHaversine(destinationPoint)
}

func (alg *AdjacencyListGraph) SetArcFlags(id NodeId, flag bool) {
	// set the arc flags for the outgoing edges
	for _, arc := range alg.GetArcsFrom(id) {
		arc.SetArcFlag(flag)
		//fmt.Printf("set arc %v -> %v: %t\n", nodeId, arc.Destination(), flag)
	}
}

func (alg *AdjacencyListGraph) EnableAllArcs() {
	for i := range alg.GetNodes() {
		alg.SetArcFlags(i, true)
	}
}
