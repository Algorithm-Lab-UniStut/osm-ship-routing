package routing

import (
	"log"
	"math"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/graph/path"
)

type Route struct {
	Origin      geo.Point
	Destination geo.Point
	Exists      bool        // true iff a route from origin to destination exists
	Waypoints   []geo.Point // sequence of points that describe the route
	Length      int         // length of the route
}

type ShipRouter struct {
	g               graph.Graph
	contractedGraph graph.Graph
	shortcuts       []path.Shortcut
	nodeOrdering    [][]int
	navigator       path.Navigator
}

func NewShipRouter(g, contractedGraph graph.Graph, shortcuts []path.Shortcut, nodeOrdering [][]int, navigator *string) *ShipRouter {
	sr := &ShipRouter{g: g, contractedGraph: contractedGraph, shortcuts: shortcuts, nodeOrdering: nodeOrdering}
	if navigator != nil {
		// overwrite navigator
		if !sr.SetNavigator(*navigator) {
			log.Fatal("Could not set navigator")
		}
	}
	return sr
}

func (sr ShipRouter) closestNodes(p1, p2 geo.Point) (n1, n2 int) {
	n1, n2 = 0, 0
	d1, d2 := math.MaxInt, math.MaxInt

	for i := 0; i < sr.g.NodeCount(); i++ {
		testPoint := sr.g.GetNode(i)
		distance := p1.IntHaversine(testPoint)
		if distance < d1 {
			n1 = i
			d1 = distance
		}
		distance = p2.IntHaversine(testPoint)
		if distance < d2 {
			n2 = i
			d2 = distance
		}
	}
	return n1, n2
}

func (sr ShipRouter) ComputeRoute(origin, destination geo.Point) (route Route) {
	originNode, desdestinationNode := sr.closestNodes(origin, destination)
	length := sr.navigator.ComputeShortestPath(originNode, desdestinationNode)
	nodePath := sr.navigator.GetPath(originNode, desdestinationNode)

	if length > -1 {
		// shortest path exists
		waypoints := make([]geo.Point, 0)
		for _, nodeId := range nodePath {
			waypoints = append(waypoints, *sr.g.GetNode(nodeId))
		}
		route = Route{Origin: origin, Destination: destination, Exists: true, Waypoints: waypoints, Length: length}
	} else {
		// shortest path does not exist
		route = Route{Origin: origin, Destination: destination, Exists: false}
	}
	return route
}

func (sr ShipRouter) GetNodes() []geo.Point {
	return sr.g.GetNodes()
}

func (sr ShipRouter) GetSearchSpace() []geo.Point {
	nodes := sr.navigator.GetSearchSpace()
	waypoints := make([]geo.Point, 0)
	for _, nodeItem := range nodes {
		node := sr.g.GetNode(nodeItem.NodeId())
		waypoints = append(waypoints, *node)
	}
	return waypoints
}

func (sr *ShipRouter) SetNavigator(navigator string) bool {
	switch navigator {
	case "dijkstra":
		sr.navigator = path.NewUniversalDijkstra(sr.g)
		return true
	case "astar":
		astar := path.NewUniversalDijkstra(sr.g)
		astar.SetUseHeuristic(true)
		sr.navigator = astar
		return true
	case "bidirectional-dijkstra":
		bidijkstra := path.NewUniversalDijkstra(sr.g)
		bidijkstra.SetBidirectional(true)
		sr.navigator = bidijkstra
		return true
	case "contraction-hierarchies":
		dijkstra := path.NewUniversalDijkstra(sr.contractedGraph)
		ch := path.NewContractionHierarchiesInitialized(sr.contractedGraph, dijkstra, sr.shortcuts, sr.nodeOrdering, path.MakeDefaultPathFindingOptions()) // use default path finding options
		sr.navigator = ch
		return true
	case "alt":
		return false
	}
	return false
}
