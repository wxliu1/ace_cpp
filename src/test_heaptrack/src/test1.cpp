
#include <stdlib.h>
#include <stdio.h>

int main()
{
        int *arr = (int *)malloc(sizeof(int) * 10);
        printf("After allocation %p\n", arr);

        int *p = new int;

        arr = NULL;
        return 0;
}

/*
usage:

1) setup
sudo apt-get install heaptrack
sudo apt-get install heaptrack-gui

heaptrack -v	# 查看版本号

2) run
heaptrack ./main


 */