package slice

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
