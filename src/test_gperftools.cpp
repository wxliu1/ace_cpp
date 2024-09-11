
#include <gperftools/profiler.h>
#include <iostream>

using namespace std;

void t1()
{
	int i = 0;
	while (i < 1000)
	{
		i++;
	}
}

void t2()
{

	int i = 0;
	while (i < 2000)

	{
		i++;
	}
}

void t3()
{
	for (int i = 0; i < 100000; ++i)
	{
		t1();
		t2();
	}
}

int main()
{
	ProfilerStart("test.prof");
	t3();
	printf("OK!\n");
	ProfilerStop();

	return 0;
}
