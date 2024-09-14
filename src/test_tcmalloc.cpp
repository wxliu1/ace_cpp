
#include <iostream>
#include <thread>
#include <unistd.h>

#include <gperftools/heap-profiler.h>

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

int func() {
    int *p = new int(10);
    return 0;
}

int main() {
    // HeapProfilerStart("leak_test");
    std::cout << "memory leak test" << std::endl;
    func();

    leak_memory();
    int* p2 = (int *)malloc(sizeof(int) * 10);

    bool bl2 = false;
    if(bl2)
    {
      int* p3 = (int *)malloc(sizeof(int) * 10);
    }

    for(int i = 0; i < 28; i++)
    {
      char *pch = new char;
    }
    
    getchar();

    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command, nullptr);
    // keyboard_command_process.join(); // terminate called without an active exception

    while(bl)
    {
        usleep(1000);
    }

    keyboard_command_process.join();
    // keyboard_command_process.detach();


    // HeapProfilerStop();

    return 0;
}
