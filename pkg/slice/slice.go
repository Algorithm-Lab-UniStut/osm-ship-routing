package slice

func ReverseIntInPlace(s []int) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

func Contains(s []int, value int) bool {
	for _, a := range s {
		if a == value {
			return true
		}
	}
	return false
}

func InsertInt(s []int, index, value int) []int {
	s = append(s[:index+1], s[index:]...)
	s[index] = value
	return s
}

func CompareInt(s1 []int, s2 []int) int {
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
