// SPDX-License-Identifier: MIT

package openapi_server

import (
	"encoding/json"
	"net/http"
	"strings"
)

// DefaultApiController binds http requests to an api service and writes the service results to the http response
type DefaultApiController struct {
	service      DefaultApiServicer
	errorHandler ErrorHandler
}

// DefaultApiOption for how the controller is set up.
type DefaultApiOption func(*DefaultApiController)

// WithDefaultApiErrorHandler inject ErrorHandler into controller
func WithDefaultApiErrorHandler(h ErrorHandler) DefaultApiOption {
	return func(c *DefaultApiController) {
		c.errorHandler = h
	}
}

// NewDefaultApiController creates a default api controller
func NewDefaultApiController(s DefaultApiServicer, opts ...DefaultApiOption) Router {
	controller := &DefaultApiController{
		service:      s,
		errorHandler: DefaultErrorHandler,
	}

	for _, opt := range opts {
		opt(controller)
	}

	return controller
}

// Routes returns all of the api route for the DefaultApiController
func (c *DefaultApiController) Routes() Routes {
	return Routes{
		{
			"ComputeRoute",
			strings.ToUpper("Post"),
			"/routes",
			c.ComputeRoute,
		},
		{
			"GetNodes",
			strings.ToUpper("Get"),
			"/nodes",
			c.GetNodes,
		},
		{
			"GetSearchSpace",
			strings.ToUpper("Get"),
			"/searchSpace",
			c.GetSearchSpace,
		},
		{
			"SetNavigator",
			strings.ToUpper("Post"),
			"/navigator",
			c.SetNavigator,
		},
	}
}

// ComputeRoute - Compute a new route
func (c *DefaultApiController) ComputeRoute(w http.ResponseWriter, r *http.Request) {
	routeRequestParam := RouteRequest{}
	d := json.NewDecoder(r.Body)
	d.DisallowUnknownFields()
	if err := d.Decode(&routeRequestParam); err != nil {
		c.errorHandler(w, r, &ParsingError{Err: err}, nil)
		return
	}
	if err := AssertRouteRequestRequired(routeRequestParam); err != nil {
		c.errorHandler(w, r, err, nil)
		return
	}
	result, err := c.service.ComputeRoute(r.Context(), routeRequestParam)
	// If an error occurred, encode the error with the status code
	if err != nil {
		c.errorHandler(w, r, err, &result)
		return
	}
	// If no error, encode the body and the result code
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "POST")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
	EncodeJSONResponse(result.Body, &result.Code, w)
}

func (c *DefaultApiController) GetNodes(w http.ResponseWriter, r *http.Request) {
	result, err := c.service.GetNodes(r.Context())
	// If an error occurred, encode the error with the status code
	if err != nil {
		c.errorHandler(w, r, err, &result)
		return
	}
	// If no error, encode the body and the result code
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
	EncodeJSONResponse(result.Body, &result.Code, w)
}

func (c *DefaultApiController) GetSearchSpace(w http.ResponseWriter, r *http.Request) {
	result, err := c.service.GetSearchSpace(r.Context())
	// If an error occurred, encode the error with the status code
	if err != nil {
		c.errorHandler(w, r, err, &result)
		return
	}
	// If no error, encode the body and the result code
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
	EncodeJSONResponse(result.Body, &result.Code, w)
}

func (c *DefaultApiController) SetNavigator(w http.ResponseWriter, r *http.Request) {
	navigatorRequestParam := NavigatorRequest{}
	d := json.NewDecoder(r.Body)
	d.DisallowUnknownFields()
	if err := d.Decode(&navigatorRequestParam); err != nil {
		c.errorHandler(w, r, &ParsingError{Err: err}, nil)
		return
	}
	if err := AssertNavigatorRequestRequired(navigatorRequestParam); err != nil {
		c.errorHandler(w, r, err, nil)
		return
	}
	result, err := c.service.SetNavigator(r.Context(), navigatorRequestParam)
	// If an error occurred, encode the error with the status code
	if err != nil {
		c.errorHandler(w, r, err, &result)
		return
	}
	// If no error, encode the body and the result code
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Header().Set("Access-Control-Allow-Methods", "GET")
	w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
	EncodeJSONResponse(result.Body, &result.Code, w)
}
