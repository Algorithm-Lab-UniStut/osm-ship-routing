package path

// Provides different options how the NodeOrder can get computed
type OrderOptions byte

const (
	random                     OrderOptions = 1 << iota // use a random initial order
	considerEdgeDifference                              // consider the edge difference when computing the order
	considerProcessedNeighbors                          // consider the processed neighbors (spatial diversity) when computing the order
	lazyUpdate                                          // enable recomputation of the order before every contraction step.
	updateNeighbors                                     // update the neighbors of a contracted node
	periodic                                            // periodically update the whole order
)

// Create a new OrderOptions (which in initially empty)
func MakeOrderOptions() OrderOptions {
	return OrderOptions(0)
}

// Set the defined options and return a new OrderOptions
func (oo OrderOptions) Set(o OrderOptions) OrderOptions {
	return oo | o
}

// Reset the defined options and return a new OrderOptions
func (oo OrderOptions) Reset(o OrderOptions) OrderOptions {
	return oo & ^o
}

// Set the dynamic option and return a new OrderOptions
func (oo OrderOptions) SetLazyUpdate(flag bool) OrderOptions {
	if flag {
		return oo.Set(lazyUpdate)
	} else {
		return oo.Reset(lazyUpdate)
	}
}

func (oo OrderOptions) IsLazyUpdate() bool {
	return oo&lazyUpdate != 0
}

func (oo OrderOptions) SetRandom(flag bool) OrderOptions {
	if flag {
		return oo.Set(random)
	} else {
		return oo.Reset(random)
	}
}

func (oo OrderOptions) IsRandom() bool {
	return oo&random != 0
}

func (oo OrderOptions) SetEdgeDifference(flag bool) OrderOptions {
	if flag {
		return oo.Set(considerEdgeDifference)
	} else {
		return oo.Reset(considerEdgeDifference)
	}
}

func (oo OrderOptions) ConsiderEdgeDifference() bool {
	return oo&considerEdgeDifference != 0
}

func (oo OrderOptions) SetProcessedNeighbors(flag bool) OrderOptions {
	if flag {
		return oo.Set(considerProcessedNeighbors)
	} else {
		return oo.Reset(considerProcessedNeighbors)
	}
}

func (oo OrderOptions) ConsiderProcessedNeighbors() bool {
	return oo&considerProcessedNeighbors != 0
}

func (oo OrderOptions) SetPeriodic(flag bool) OrderOptions {
	if flag {
		return oo.Set(periodic)
	} else {
		return oo.Reset(periodic)
	}
}

func (oo OrderOptions) IsPeriodic() bool {
	return oo&periodic != 0
}

func (oo OrderOptions) SetUpdateNeighbors(flag bool) OrderOptions {
	if flag {
		return oo.Set(updateNeighbors)
	} else {
		return oo.Reset(updateNeighbors)
	}
}

func (oo OrderOptions) UpdateNeighbors() bool {
	return oo&updateNeighbors != 0
}

func (oo OrderOptions) IsValid() bool {
	if !oo.IsRandom() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using no random order, either the edge difference or the processed neighbors is needed for initial order computation
		return false
	}
	if oo.IsLazyUpdate() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using dynamic order, either the edge difference or the processed neighbors (or both) must get considered
		return false
	}
	return true
}
