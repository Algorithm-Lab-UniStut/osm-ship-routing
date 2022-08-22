package graph

import (
	"fmt"
	"strings"
)

type NodeId = int

type Arc interface {
	Destination() NodeId
	Cost() int
	ArcFlag() bool
	SetArcFlag(flag bool)
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
	EstimateDistance(source, target NodeId) int
}

type DynamicGraph interface {
	Graph
	AddNode(n Node)
	AddEdge(edge Edge)
}

type Node struct {
	Lon float64
	Lat float64
	// TODO: id?
	// TODO: Point attribute/ implement Point type
}

type Edge struct {
	From     NodeId
	To       NodeId
	Distance int
	arcFlag  bool
}

type OutgoingEdge struct {
	To       NodeId
	Distance int
	arcFlag  bool
}

type OutgoingEdges = []OutgoingEdge

func NewNode(lon float64, lat float64) *Node {
	return &Node{Lon: lon, Lat: lat}
}

func NewEdge(to NodeId, from NodeId, distance int) *Edge {
	return &Edge{To: to, From: from, Distance: distance, arcFlag: true}
}

func NewOutgoingEdge(to NodeId, distance int, arcFlag bool) *OutgoingEdge {
	return &OutgoingEdge{To: to, Distance: distance, arcFlag: arcFlag}
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

func (e Edge) toOutgoingEdge() *OutgoingEdge {
	return NewOutgoingEdge(e.To, e.Distance, e.ArcFlag()) //OutgoingEdge{To: e.To, Distance: e.Distance, arcFlag: e.ArcFlag()}
}

func (e Edge) ArcFlag() bool {
	return e.arcFlag
}

func (e *Edge) SetArcFlag(flag bool) {
	e.arcFlag = flag
}

func (oe OutgoingEdge) toEdge(from NodeId) Edge {
	return Edge{From: from, To: oe.To, Distance: oe.Distance, arcFlag: oe.ArcFlag()}
}

func (oe OutgoingEdge) Destination() NodeId {
	return oe.To
}

func (oe OutgoingEdge) Cost() int {
	return oe.Distance
}

func (oe OutgoingEdge) ArcFlag() bool { return oe.arcFlag }

func (oe *OutgoingEdge) SetArcFlag(flag bool) { oe.arcFlag = flag }

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
