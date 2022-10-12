// SPDX-License-Identifier: MIT

package openapi_server

import (
	"context"
	"net/http"
	"sync"

	"github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/graph/path"
	"github.com/natevvv/osm-ship-routing/pkg/routing"
)

// DefaultApiService is a service that implements the logic for the DefaultApiServicer
// This service should implement the business logic for every endpoint for the DefaultApi API.
// Include any external packages or services that will be required by this service.
type DefaultApiService struct {
	shipRouter *routing.ShipRouter
}

// NewDefaultApiService creates a default api service
func NewDefaultApiService(graphFile, contractedGraphFile, shortcutFile, nodeOrderingFile string, navigator *string) DefaultApiServicer {
	var g graph.Graph
	var contractedGraph graph.Graph
	var shortcuts []path.Shortcut
	var nodeOrdering [][]int

	var wg sync.WaitGroup
	wg.Add(4)
	go func() {
		g = graph.NewAdjacencyArrayFromFmiFile(graphFile)
		wg.Done()
	}()
	go func() {
		contractedGraph = graph.NewAdjacencyArrayFromFmiFile(contractedGraphFile)
		wg.Done()
	}()
	go func() {
		shortcuts = path.ReadShortcutFile(shortcutFile)
		wg.Done()
	}()
	go func() {
		nodeOrdering = path.ReadNodeOrderingFile(nodeOrderingFile)
		wg.Done()
	}()
	wg.Wait()

	sr := routing.NewShipRouter(g, contractedGraph, shortcuts, nodeOrdering, navigator)
	return &DefaultApiService{shipRouter: sr}
}

// ComputeRoute - Compute a new route
func (s *DefaultApiService) ComputeRoute(ctx context.Context, routeRequest RouteRequest) (ImplResponse, error) {
	origin := geometry.NewPoint(float64(routeRequest.Origin.Lat), float64(routeRequest.Origin.Lon))
	destination := geometry.NewPoint(float64(routeRequest.Destination.Lat), float64(routeRequest.Destination.Lon))

	route := s.shipRouter.ComputeRoute(*origin, *destination)

	routeResult := RouteResult{Origin: routeRequest.Origin, Destination: routeRequest.Destination}
	if route.Exists {
		routeResult.Reachable = true
		waypoints := make([]Point, 0)
		for _, waypoint := range route.Waypoints {
			p := Point{Lat: float32(waypoint.Lat()), Lon: float32(waypoint.Lon())}
			waypoints = append(waypoints, p)
		}
		routeResult.Path = Path{Length: int32(route.Length), Waypoints: waypoints}
	} else {
		routeResult.Reachable = false
	}

	return Response(http.StatusOK, routeResult), nil
}

func (s *DefaultApiService) GetNodes(ctx context.Context) (ImplResponse, error) {
	points := s.shipRouter.GetNodes()

	vertices := make([]Point, 0)
	for _, point := range points {
		p := Point{Lat: float32(point.Lat()), Lon: float32(point.Lon())}
		vertices = append(vertices, p)
	}
	nodes := Nodes{Waypoints: vertices}

	return Response(http.StatusOK, nodes), nil
}

func (s *DefaultApiService) GetSearchSpace(ctx context.Context) (ImplResponse, error) {
	points := s.shipRouter.GetSearchSpace()

	vertices := make([]Point, 0)
	for _, point := range points {
		p := Point{Lat: float32(point.Lat()), Lon: float32(point.Lon())}
		vertices = append(vertices, p)
	}
	nodes := Nodes{Waypoints: vertices}

	return Response(http.StatusOK, nodes), nil
}

func (s *DefaultApiService) SetNavigator(ctx context.Context, navigatorRequest NavigatorRequest) (ImplResponse, error) {
	success := s.shipRouter.SetNavigator(navigatorRequest.Navigator)

	if !success {
		return Response(http.StatusBadRequest, "Unknown Navigator"), nil
	}
	return Response(http.StatusOK, navigatorRequest.Navigator), nil
}
