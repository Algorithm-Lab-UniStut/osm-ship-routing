// SPDX-License-Identifier: MIT

package openapi_server

type NavigatorRequest struct {
	Navigator string `json:"navigator"`
}

func AssertNavigatorRequestRequired(obj NavigatorRequest) error {
	elements := map[string]interface{}{
		"navigator": obj.Navigator,
	}
	for name, el := range elements {
		if isZero := IsZeroValue(el); isZero {
			return &RequiredError{Field: name}
		}
	}
	return nil
}
