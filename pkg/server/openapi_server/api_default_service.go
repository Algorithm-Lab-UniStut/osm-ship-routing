/*
 * Ship Routing API
 *
 * Access the global ship routing service via a RESTful API
 *
 * API version: 0.0.1
 * Generated by: OpenAPI Generator (https://openapi-generator.tech)
 */

package openapi_server

import (
	"context"
	"net/http"

	"github.com/dmholtz/osm-ship-routing/pkg/geometry"
	"github.com/dmholtz/osm-ship-routing/pkg/graph"
)

// DefaultApiService is a service that implements the logic for the DefaultApiServicer
// This service should implement the business logic for every endpoint for the DefaultApi API.
// Include any external packages or services that will be required by this service.
type DefaultApiService struct {
	shipRouter *graph.ShipRouter
}

// NewDefaultApiService creates a default api service
func NewDefaultApiService() DefaultApiServicer {
	g := graph.NewAdjacencyArrayFromFmi("ocean_1M.fmi")
	sr := graph.NewShipRouter(g)
	return &DefaultApiService{shipRouter: sr}
}

// ComputeRoute - Compute a new route
func (s *DefaultApiService) ComputeRoute(ctx context.Context, routeRequest RouteRequest) (ImplResponse, error) {
	// TODO - update ComputeRoute with the required logic for this service method.
	// Add api_default_service.go to the .openapi-generator-ignore to avoid overwriting this service implementation when updating open api generation.
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

	//TODO: Uncomment the next line to return response Response(200, RouteResult{}) or use other options such as http.Ok ...
	return Response(http.StatusOK, routeResult), nil
}