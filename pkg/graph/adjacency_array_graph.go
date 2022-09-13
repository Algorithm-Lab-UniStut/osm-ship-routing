package graph

import (
	"fmt"
	"strings"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
)

// Implementation for static graphs
type AdjacencyArrayGraph struct {
	Nodes   []Node
	arcs    []*Arc
	Offsets []int
}

// Create an AdjacencyArrayGraph from the given graph
func NewAdjacencyArrayFromGraph(g Graph) *AdjacencyArrayGraph {
	nodes := make([]Node, 0)
	arcs := make([]*Arc, 0)
	offsets := make([]int, g.NodeCount()+1, g.NodeCount()+1)

	for i := 0; i < g.NodeCount(); i++ {
		// add node
		nodes = append(nodes, g.GetNode(i))

		// add all edges of node
		for _, arc := range g.GetArcsFrom(i) {
			arcs = append(arcs, arc)
		}

		// set stop-offset
		offsets[i+1] = len(arcs)
	}

	aag := AdjacencyArrayGraph{Nodes: nodes, arcs: arcs, Offsets: offsets}
	return &aag
}

// Get the node for the given id
func (aag *AdjacencyArrayGraph) GetNode(id NodeId) Node {
	if id < 0 || id >= aag.NodeCount() {
		panic(fmt.Sprintf("NodeId %d is not contained in the graph.", id))
	}
	return aag.Nodes[id]
}

// get all nodes of the graph
func (aag *AdjacencyArrayGraph) GetNodes() []Node {
	return aag.Nodes
}

// Get the Arcs for the given node id
func (aag *AdjacencyArrayGraph) GetArcsFrom(id NodeId) []*Arc {
	if id < 0 || id >= aag.NodeCount() {
		panic(fmt.Sprintf("NodeId %d is not contained in the graph.", id))
	}
	return aag.arcs[aag.Offsets[id]:aag.Offsets[id+1]]
}

// Returns the number of Nodes in the graph
func (aag *AdjacencyArrayGraph) NodeCount() int {
	return len(aag.Nodes)
}

/*
func (aag *AdjacencyArrayGraph) EdgeCount() int {
	return len(aag.Arcs)
}
*/

// Returns the total number of arcs in the graph
func (aag *AdjacencyArrayGraph) ArcCount() int {
	return len(aag.arcs)
}

// Returns a human readable string of the graph
func (aag *AdjacencyArrayGraph) AsString() string {
	var sb strings.Builder

	// write number of nodes and number of edges
	sb.WriteString(fmt.Sprintf("%v\n", aag.NodeCount()))
	sb.WriteString(fmt.Sprintf("%v\n", aag.ArcCount()))

	sb.WriteString(fmt.Sprintf("#Nodes\n"))
	// list all nodes structured as "id lat lon"
	for i := 0; i < aag.NodeCount(); i++ {
		node := aag.GetNode(i)
		sb.WriteString(fmt.Sprintf("%v %v %v\n", i, node.Lat, node.Lon))
	}

	sb.WriteString(fmt.Sprintf("#Edges\n"))
	// list all edges structured as "fromId targetId distance"
	for i := 0; i < aag.NodeCount(); i++ {
		for _, arc := range aag.GetArcsFrom(i) {
			sb.WriteString(fmt.Sprintf("%v %v %v\n", i, arc.Destination(), arc.Cost()))
		}
	}
	return sb.String()
}

// Estimate the distance between the given nodes
// This calculates the direct distance (air line, bird path length) between the nodes
func (aag *AdjacencyArrayGraph) EstimateDistance(source, target NodeId) int {
	origin := aag.GetNode(source)
	destination := aag.GetNode(target)
	originPoint := geo.NewPoint(origin.Lat, origin.Lon)
	destinationPoint := geo.NewPoint(destination.Lat, destination.Lon)
	return int(0.99 * float64(originPoint.IntHaversine(destinationPoint)))
}

// Set the arc flags for all arcs of the given node
func (aag *AdjacencyArrayGraph) SetArcFlags(id NodeId, flag bool) {
	// set the arc flags for the outgoing edges
	for _, arc := range aag.GetArcsFrom(id) {
		arc.SetArcFlag(flag)
		//fmt.Printf("set arc %v -> %v: %t\n", nodeId, arc.Destination(), flag)
	}
	// maybe this can get improved to directly set the arcs flags without calling GetArcsFrom()
}

// Set the arc flag for the given arc
func (aag *AdjacencyArrayGraph) SetArcFlag(id NodeId, arcIndex int, flag bool) {
	panic("Not implemented")
}

// Enable all arcs in the graph
func (aag *AdjacencyArrayGraph) EnableAllArcs() {
	for i := range aag.GetNodes() {
		aag.SetArcFlags(i, true)
	}
}
