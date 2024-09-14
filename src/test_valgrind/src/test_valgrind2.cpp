
#include <cstdint>
#include <thread>
#include <chrono>
#include <vector>

using namespace std;

vector<uint32_t> g_memPool;

void Fun(uint32_t num)
{
    g_memPool.push_back(num);
}

int32_t main()
{
    for (uint8_t i = 0; i < 256; ++i) {
        Fun(i);
        this_thread::sleep_for(chrono::milliseconds(1));   // 我们为了统计结果更直观，此处将延时改为1ms
    }

    return 0;
}
