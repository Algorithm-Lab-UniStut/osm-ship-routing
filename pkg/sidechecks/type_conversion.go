package sidechecks

import "fmt"

type MyInt int

func processMyInt(mi MyInt) {
	fmt.Println(mi)
}

func processInt(i int) {
	fmt.Println(i)
}

func TypeConversion() {
	var i int = 4
	var mi MyInt = 4
	if MyInt(i) == mi {
		fmt.Println("Same")
	}
	processMyInt(mi)
	processMyInt(MyInt(i))
	processInt(int(mi))
	processInt(i)
	//mi = i
}
