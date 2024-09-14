#include <thread>
#include <chrono>
using namespace std;

int main()
{
    int* ptr = new int[10240]();

    return 0;
}


// usage:
// valgrind --tool=massif --time-unit=ms ./Xxxx
// massif-visualizer massif.out.1132

