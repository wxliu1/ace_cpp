
#include <cstdint>
#include <thread>
#include <chrono>
#include <vector>
#include <thread>
#include <iostream>
#include <signal.h>

using namespace std;

vector<uint32_t> g_memPool;

bool bl = true;

void command(void *pParam)
{
    // int *p = new int();
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

// 2023-11-11
void sigintHandler(int sig) {
    
    printf("sig=%d\n",sig);
    bl = false;
    signal(sig,SIG_DFL);//设置收到信号SIGINT采取默认方式响应（ctrl+c~结束进程）
    
    // 执行特定的中断处理操作
    // 例如，关闭打开的文件、释放资源等
    // 然后退出程序

    // exit(0); // tmp comment.
}

void Fun(uint32_t num)
{
    g_memPool.push_back(num);
}

int32_t main(int argc, char** argv)
{
    // 注册中断信号处理函数
    signal(SIGINT, sigintHandler); // 2023-11-11

    // std::thread keyboard_command_process;
    // keyboard_command_process = std::thread(command, nullptr);

    int* p2 = (int *)malloc(sizeof(int) * 10);


    for (uint8_t i = 0; i < 256; ++i) {
        Fun(i);
        this_thread::sleep_for(chrono::milliseconds(1));   // 我们为了统计结果更直观，此处将延时改为1ms
        if(!bl) break ;
    }
/**/

    // keyboard_command_process.join();

    std::cout << "main() exit\n";

    return 0;
}