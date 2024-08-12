
// test tbb::concurrent_bounded_queue::emplace

#include <tbb/concurrent_queue.h>
#include <iostream>

int main(int argc, char *argv[])
{
    
    tbb::concurrent_bounded_queue<int> q;
    q.push(6);
    q.emplace(8);

    auto i = q.unsafe_begin();
    std::cout << "*i=" << *i << std::endl;
    i++;
    std::cout << "*i=" << *i << std::endl;

    while(!q.empty())
    {
        int x;
        q.pop(x);

        std::cout << x << std::endl; 
    }

    return 0;
}