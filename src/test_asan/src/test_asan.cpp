
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <iostream>
#include <thread>

bool bl = true;

void command(void *pParam)
{
  std::cout << "1 command()" << std::endl;
  while (bl)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    
    if(c == 'q')
    {
        std::cout << "press 'q' to quit" << std::endl;
    // stop();
    // ros::shutdown();
        bl = false;
    }

    

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

void leak_memory() {
    int* p = (int *)malloc(sizeof(int) * 10);
    // 没有释放内存，导致内存泄漏
}

int main() {
/*
    int* arr = (int *)malloc(sizeof(int) * 5);
    arr[5] = 10; // 内存访问越界错误

    free(arr); // 使用 free 释放内存

    int* p = (int *)malloc(sizeof(int));
    free(p); // 使用 free 释放内存

    int* q = NULL;
    *q = 5; // 使用空指针访问内存错误
*/
    leak_memory();
    int* p2 = (int *)malloc(sizeof(int) * 10);
    
    getchar();

    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command, nullptr);
    // keyboard_command_process.join(); // terminate called without an active exception

    while(bl)
    {
        usleep(1000);
    }

    keyboard_command_process.join();

    return 0;
}
