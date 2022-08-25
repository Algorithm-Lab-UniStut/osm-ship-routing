package path

type OrderOptions byte

const (
	dynamic OrderOptions = 1 << iota
	random
	considerEdgeDifference
	considerProcessedNeighbors
	periodic
)

func MakeOrderOptions() OrderOptions {
	return OrderOptions(0)
}

func (oo OrderOptions) Set(o OrderOptions) OrderOptions {
	return oo | o
}

func (oo OrderOptions) Reset(o OrderOptions) OrderOptions {
	return oo & ^o
}

func (oo OrderOptions) SetDynamic(flag bool) OrderOptions {
	if flag {
		return oo.Set(dynamic)
	} else {
		return oo.Reset(dynamic)
	}
}

func (oo OrderOptions) IsDynamic() bool {
	return oo&dynamic != 0
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

func (oo OrderOptions) IsValid() bool {
	if !oo.IsRandom() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using no random order, either the edge difference or the processed neighbors is needed for initial order computation
		return false
	}
	if oo.IsDynamic() && !(oo.ConsiderEdgeDifference() || oo.ConsiderProcessedNeighbors()) {
		// if using dynamic order, either the edge difference or the processed neighbors (or both) must get considered
		return false
	}
	return true
}
