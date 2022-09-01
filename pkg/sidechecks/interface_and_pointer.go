package sidechecks

import "fmt"

type Increaser interface {
	Increase()
	Value() int
}

type Valuer interface {
	Value() int
}

type Inc struct {
	a int
}

func (i *Inc) Increase() {
	i.a++
}

func (i Inc) Value() int {
	return i.a
}

func PrintValue(val Valuer) {
	fmt.Printf("%v\n", val.Value())
	fmt.Printf("Address of given interface: %p\n", &val)
}

func InterfaceAndPointer() {
	var it Increaser = &Inc{}
	var i = &Inc{a: 3}
	var test = 4
	fmt.Printf("Address of test (stack): %p\n", &test)
	fmt.Printf("Address of it: %p\n", &it)
	fmt.Printf("Address of i: %p\n", &i)
	it.Increase()
	i.Increase()
	PrintValue(it)
	PrintValue(i)
	var inc = &Inc{}
	var inc2 Increaser = inc
	inc.Increase()
	PrintValue(inc)
	PrintValue(inc2)
	var val = Inc{a: 2}
	var ival Valuer = val
	val.a = 5
	PrintValue(val)
	PrintValue(ival)
}
