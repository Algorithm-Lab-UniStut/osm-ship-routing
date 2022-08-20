package graph

import (
	"fmt"
	"strings"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
)

// Implementation for dynamic graphs
type AdjacencyListGraph struct {
	Nodes     []Node
	Edges     [][]OutgoingEdge // TODO: use interface Arc
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

func (alg *AdjacencyListGraph) GetArcsFrom(id NodeId) []Arc {
	if id < 0 || id >= alg.NodeCount() {
		panic(id)
	}
	arcs := make([]Arc, 0)
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

	// list all nodes structured as "id lat lon"
	for i := 0; i < alg.NodeCount(); i++ {
		node := alg.GetNode(i)
		sb.WriteString(fmt.Sprintf("%v %v %v\n", i, node.Lat, node.Lon))
	}

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
	alg.Edges = append(alg.Edges, make([]OutgoingEdge, 0))
}

func (alg *AdjacencyListGraph) AddArc(e Edge) {
	// Check if both source and target node exit
	if e.From >= alg.NodeCount() || e.To >= alg.NodeCount() {
		panic(fmt.Sprintf("Edge out of range %v", e))
	}
	// Check for duplicates
	for _, outgoingEdge := range alg.Edges[e.From] {
		if e.To == outgoingEdge.To {
			return // ignore duplicate edges
		}
	}
	alg.Edges[e.From] = append(alg.Edges[e.From], e.toOutgoingEdge())
	alg.edgeCount++
}

func (alg *AdjacencyListGraph) EstimateDistance(source, target NodeId) int {
	origin := alg.GetNode(source)
	destination := alg.GetNode(target)
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return originPoint.IntHaversine(destinationPoint)
}
