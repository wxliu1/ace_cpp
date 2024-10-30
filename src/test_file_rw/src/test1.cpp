#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

void read_meminfo()
{
    std::ifstream file("/proc/meminfo");
    if (file.is_open())
    {
        long MemAvailable = 0;
        std::string line;
        while (std::getline(file, line))
        {
            if (line.find("MemAvailable") != std::string::npos)
            {
                size_t colonPos = line.find(':');
                if (colonPos != std::string::npos)
                {
                    MemAvailable = std::stol(line.substr(colonPos + 1));
                    std::cout << "memory avaliable: " << MemAvailable << std::endl;
                }
                break;
            }
        }
        file.close();

        if (MemAvailable < 512 * 1024)
        {
            std::chrono::milliseconds dura(500);
            std::this_thread::sleep_for(dura);
            // continue;
        }
    }
}

void process()
{
    while(true)
    {
        read_meminfo();

        std::chrono::milliseconds dura(5); // 5
        std::this_thread::sleep_for(dura);
    }
}

void command(void *pParam)
{

  std::cout << "1 command()" << std::endl;
  while (true)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    
    if(c == 'v')
    {
        std::cout << "press 'v' to load vocabulary" << std::endl;
        // loadVocabulary(vocabulary_file);
    }

    else if(c == 'p')
    {
        std::cout << "press 'p' to loadPoseGrpah" << std::endl;
        // loadPoseGraph();
    }

    else if(c == 'r')
    {
        std::cout << "press 'r' to reset" << std::endl;
        // Reset();
    }
    else if(c == 'q')
    {
        std::cout << "press 'q' to quit" << std::endl;
        // Stop();
        // shutdown = true;
        break ;

    }

    else if(c == 'm')
    {
        std::cout << "press 'm' to read meminfo" << std::endl;
        read_meminfo();
    }

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << "ch=" << getchar() << std::endl;
    std::thread t_process;
    // t_process = std::thread(process, nullptr);
    t_process = std::thread(process);
    t_process.join();

    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command, nullptr);
    keyboard_command_process.join(); // terminate called without an active exception

    return 0;
}