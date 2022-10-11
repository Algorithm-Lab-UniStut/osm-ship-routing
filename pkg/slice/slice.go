package slice

type FixedSizeSlice struct {
	slice        []bool
	numSetValues int
}

func MakeFixedSizeSlice(length int) FixedSizeSlice {
	return FixedSizeSlice{slice: make([]bool, length), numSetValues: 0}
}
func (s *FixedSizeSlice) Len() int { return s.numSetValues }
func (s *FixedSizeSlice) Add(indices ...int) {
	for _, index := range indices {
		if !s.slice[index] {
			s.slice[index] = true
			s.numSetValues++
		}
	}
}

func (s *FixedSizeSlice) Remove(indices ...int) {
	for _, index := range indices {
		if s.slice[index] {
			s.slice[index] = false
			s.numSetValues--
		}
	}
}

func (s *FixedSizeSlice) Get() []bool    { return s.slice }
func (s *FixedSizeSlice) Ratio() float64 { return float64(s.numSetValues) / float64(len(s.slice)) }

func ReverseInPlace[T any](s []T) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

func Contains[T comparable](s []T, value T) bool {
	for _, a := range s {
		if a == value {
			return true
		}
	}
	return false
}

func Insert[T any](s []T, index int, value T) []T {
	s = append(s[:index+1], s[index:]...)
	s[index] = value
	return s
}

func Compare[T comparable](s1 []T, s2 []T) int {
	if len(s1) != len(s2) {
		return -1
	}
	differences := 0
	for i := 0; i < len(s1); i++ {
		if s1[i] != s2[i] {
			differences++
		}
	}
	return differences
}
