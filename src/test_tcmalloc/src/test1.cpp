
#include <iostream>
using namespace std;
int main()
{
        int *p = new int();
        return 0;
}

/*
 usage:
 method 1 : 不能定位到具体行
env HEAPCHECK=normal ./test1

method 2: 
export HEAPPROFILE=test1
./test1
pprof ./test1 "test1.0001.heap" --inuse_objects --lines --heapcheck  --edgefraction=1e-10 --nodefraction=1e-10 --text
 */