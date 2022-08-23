package sidechecks

import "fmt"

type Test struct {
	a int
	b int
}

type Test2 struct {
	a int
	b int
}

type ITest interface {
	Sum() int
}

func (t Test) Sum() int {
	return t.a + t.b
}

func (t *Test2) Sum() int {
	return t.a + t.b
}

func InterfaceAndOptimization() {
	//t := Test{a: 1, b: 3}
	//sum := t.Sum()
	//fmt.Printf("Sum: %v\n", sum)
	var t = &Test{a: 1, b: 3}
	var it ITest = &Test{a: 1, b: 3}
	var it2 = &Test2{a: 1, b: 3}
	isum := t.Sum()
	itsum := it.Sum()
	isum2 := it2.Sum()
	fmt.Printf("Sum: %v\n", isum)
	fmt.Printf("Sum: %v\n", itsum)
	fmt.Printf("Sum: %v\n", isum2)
}
