package openapi_server

type Nodes struct {
	Waypoints []Point `json:"waypoints"`
}

// AssertPathRequired checks if the required fields are not zero-ed
func AssertNodesRequired(obj Nodes) error {
	elements := map[string]interface{}{
		"waypoints": obj.Waypoints,
	}
	for name, el := range elements {
		if isZero := IsZeroValue(el); isZero {
			return &RequiredError{Field: name}
		}
	}

	for _, el := range obj.Waypoints {
		if err := AssertPointRequired(el); err != nil {
			return err
		}
	}
	return nil
}
