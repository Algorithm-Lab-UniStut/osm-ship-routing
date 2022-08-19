package graph

import (
	"fmt"
	"strings"
)

type NodeId = int

type Node struct {
	Lon float64
	Lat float64
}

type Edge struct {
	From     NodeId
	To       NodeId
	Distance int
}

type OutgoingEdge struct {
	To       NodeId
	Distance int
}

type OutgoingEdges = []OutgoingEdge

type Arc interface {
	Destination() NodeId
	Cost() int
}

type Graph interface {
	GetNode(id NodeId) Node
	GetNodes() []Node
	//GetEdgesFrom(id NodeId) []Edge
	GetArcsFrom(id NodeId) []Arc
	NodeCount() int
	//EdgeCount() int
	ArcCount() int
	AsString() string
	// ReduceToLargestConnectedComponent() Graph -> why is this needed?
}

type DynamicGraph interface {
	Graph
	AddNode(n Node)
	AddArc(arc Arc)
}

func NewNode(lon float64, lat float64) *Node {
	return &Node{Lon: lon, Lat: lat}
}

func NewEdge(to NodeId, from NodeId, distance int) *Edge {
	return &Edge{To: to, From: from, Distance: distance}
}

func (e Edge) Destination() NodeId {
	return e.To
}

func (e Edge) Cost() int {
	return e.Distance
}

func (e Edge) Invert() Edge {
	return Edge{From: e.To, To: e.From, Distance: e.Distance}
}

func (e Edge) toOutgoingEdge() OutgoingEdge {
	return OutgoingEdge{To: e.To, Distance: e.Distance}
}

func (oe OutgoingEdge) toEdge(from NodeId) Edge {
	return Edge{From: from, To: oe.To, Distance: oe.Distance}
}

func (oe OutgoingEdge) Destination() NodeId {
	return oe.To
}

func (oe OutgoingEdge) Cost() int {
	return oe.Distance
}

func GraphAsString(g Graph) string {
	var sb strings.Builder

	// write number of nodes and number of edges
	sb.WriteString(fmt.Sprintf("%v\n", g.NodeCount()))
	sb.WriteString(fmt.Sprintf("%v\n", g.ArcCount()))

	// list all nodes structured as "id lat lon"
	for i := 0; i < g.NodeCount(); i++ {
		node := g.GetNode(i)
		sb.WriteString(fmt.Sprintf("%v %v %v\n", i, node.Lat, node.Lon))
	}

	// list all edges structured as "fromId targetId distance"
	for i := 0; i < g.NodeCount(); i++ {
		for _, arc := range g.GetArcsFrom(i) {
			sb.WriteString(fmt.Sprintf("%v %v %v\n", i, arc.Destination(), arc.Cost()))
		}
	}
	return sb.String()
}
