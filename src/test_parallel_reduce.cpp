
// #include <tbb/blocked_range.h>
// #include <tbb/parallel_for.h>
// #include <tbb/parallel_reduce.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

using namespace std;
using namespace tbb;

void function(int i, int &local)
{
    local = i + 1;
}

int main(int argc, char *argv[])
{
    vector<int> vec;
    for (int i = 0; i < 100; i++)
        vec.push_back(i + 1);
        /*
            int result = parallel_reduce(
                blocked_range<vector<int>::iterator>(vec.begin(), vec.end()),
                0,
                [](const blocked_range<vector<int>::iterator> &r, int init) -> int
                {
                    for(auto a = r.begin(); a!=r.end(); a++)
                    init+=*a;
                    std::cout << "init=" << init << std::endl;
                    return init;
                },
                [](int x, int y) -> int
                {
                    std::cout << "x=" << x << " y=" << y << std::endl; // 不会真正执行这里
                    return x + y;
                }
            );
        */

        /*
            int result = parallel_reduce(
                blocked_range<vector<int>::iterator>(vec.begin(), vec.end(), 2),
                0,
                [](const blocked_range<vector<int>::iterator> &r, int init) -> int
                {
                    for(auto a = r.begin(); a!=r.end(); a++)
                    init+=*a;
                    std::cout << "2  init=" << init << std::endl;
                    return init;
                },
                std::plus<>{} //汇总结果
            );
        */

        /*
            int result = tbb::parallel_reduce(
                // tbb::blocked_range<int>(0, 4, 2), // 取值区间[0, 4), gsize=2
                tbb::blocked_range<int>(0, 4), // 取值区间[0, 4),默认gsize=1
                // tbb::blocked_range<int>(0, 4, 1), // 取值区间[0, 4), gsize=1
                // tbb::blocked_range<int>(0, 4, 4), // 取值区间[0, 4), gsize=4
                0,
                [](const auto& block, int init) -> int
                {
                    // for(auto a = block.begin(); a != block.end(); a++)
                    for(int i = block.begin(); i < block.end(); ++i)
                    {
                        init = i;
                        std::cout << "i = " << i << std::endl;
                    }
                    std::cout << "3  init=" << init << std::endl;
                    return init;
                },
                // std::plus<>{}
                [](int x, int y) -> int
                {
                    std::cout << "x=" << x << " y=" << y << std::endl; // 不会真正执行这里
                    return x + y;
                }
            );
        */

#if 0
    int result = tbb::parallel_reduce(
        // tbb::blocked_range<int>(0, 4, 2), // 取值区间[0, 4), gsize=2
        tbb::blocked_range<int>(0, 4), // 取值区间[0, 4),默认gsize=1
        // tbb::blocked_range<int>(0, 4, 1), // 取值区间[0, 4), gsize=1
        // tbb::blocked_range<int>(0, 4, 4), // 取值区间[0, 4), gsize=4
        0, 
        [&](const auto& block, int init) -> int
        {
            // for(auto a = block.begin(); a != block.end(); a++)
            for(int i = block.begin(); i < block.end(); ++i)
            {
               function(i, init);
                // std::cout << "i = " << i << std::endl;
            }
            std::cout << "3  init=" << init << std::endl;
            return init;
        },
        /*
        std::plus<>{}
        */
        /*
        [](int x, int y) -> int
        {
            std::cout << "x=" << x << " y=" << y << std::endl; // 不会真正执行这里
            return x + y;
        }*/

        [](int x, int y) -> int
        {
            std::cout << "x=" << x << " y=" << y << std::endl; // 不会真正执行这里
            x += y;
            return x;
        }
    );
#endif

    // cout << "result:" << result << endl;

    // auto body = [&](const tbb::blocked_range<size_t> &range,
    auto body = [](const tbb::blocked_range<size_t> &range,
                    int local)
    {
        for (size_t r = range.begin(); r != range.end(); ++r)
        {
            std::cout << "1 local=" << local << std::endl;
            // local = 1;
            local += 1;

        }
        std::cout << "local=" << local << std::endl;
        return local;
    };

    // initial_value作为初值
    int initial_value = 0;
    // join用于汇总各个线程块的结果
    auto join = [](auto p1, auto p2) { // p2为每个线程的返回值
        p1 += p2;
        std::cout << "p1=" << p1 << " p2=" << p2 << std::endl; // 这一行根本没有打印输出
        return p1; // 返回总结果
    };

    auto join2 = [](int x, int y) -> int
    {
        std::cout << "x=" << x << " y=" << y << std::endl; // 不会真正执行这里
        return x + y;
    };

    auto join3 = std::plus<>{};

    // tbb::blocked_range<size_t> range(0, 4);
    tbb::blocked_range<size_t> range(0, 4, 2);
    // auto reduction_res = tbb::parallel_reduce(range, initial_value, body, join);
    // auto reduction_res = tbb::parallel_reduce(range, initial_value, body, join2);
    // auto reduction_res = tbb::parallel_reduce(range, initial_value, body, join3);
    // auto reduction_res = tbb::parallel_reduce(range, initial_value, body, [](auto p1, auto p2) { return p1; });
    auto reduction_res = tbb::parallel_reduce(range, initial_value, body, std::plus<>{});

    std::cout << "reduction_res=" << reduction_res << std::endl;

#if 0
    //? tbb::parallel_reduce的结果不唯一??但是执行时间快
    size_t n = 1 << 26;
    size_t n0 = 1 << 0;
    size_t n1 = 1 << 1;
    size_t n2 = 1 << 2;
    size_t n3 = 1 << 3;
    std::cout << " n = " << n << std::endl;
    std::cout << " n0 = " << n0 << std::endl;
    std::cout << " n1 = " << n1 << std::endl;
    std::cout << " n2 = " << n2 << std::endl;
    std::cout << " n3 = " << n3 << std::endl;

    auto t0 = std::chrono::steady_clock::now();

    //float sum = tbb::parallel_reduce( tbb::blocked_range<size_t>(0, n), (float)0,
    //[&] ( tbb::blocked_range<size_t> r, float local_sum ) {
    //    for ( size_t i = r.begin(); i < r.end(); ++ i ) {
    //        local_sum += std::sin(i);
    //    }
    //    return local_sum;
    //}, [] (float x, float y) {
    //    return x + y;
    //});
    float sum = tbb::parallel_reduce( tbb::blocked_range<size_t>(0, n), (float)0,
        [&] ( tbb::blocked_range<size_t> r, float local_sum ) {
            for ( size_t i = r.begin(); i < r.end(); ++ i ) {
                local_sum += std::sin(i);
            }
            return local_sum;
        }, std::plus<float>()
        );
    std::cout << " sum = " << sum << std::endl;

    auto t1 = std::chrono::steady_clock::now();
    auto dt = t1 - t0;
    int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
    std::cout << "time elapsed: " << ms << " ms" << std::endl;
#endif

#if 0
    // tbb::parallel_deterministic_reduce执行结果唯一，但是速度慢
    size_t n = 1 << 26;
    size_t n0 = 1 << 0;
    size_t n1 = 1 << 1;
    size_t n2 = 1 << 2;
    size_t n3 = 1 << 3;
    std::cout << " n = " << n << std::endl;
    std::cout << " n0 = " << n0 << std::endl;
    std::cout << " n1 = " << n1 << std::endl;
    std::cout << " n2 = " << n2 << std::endl;
    std::cout << " n3 = " << n3 << std::endl;

    auto t0 = std::chrono::steady_clock::now();

    //float sum = tbb::parallel_deterministic_reduce( tbb::blocked_range<size_t>(0, n), (float)0,
    //[&] ( tbb::blocked_range<size_t> r, float local_sum ) {
    //    for ( size_t i = r.begin(); i < r.end(); ++ i ) {
    //        local_sum += std::sin(i);
    //    }
    //    return local_sum;
    //}, [] (float x, float y) {
    //    return x + y;
    //});
    float sum = tbb::parallel_deterministic_reduce( tbb::blocked_range<size_t>(0, n), (float)0,
        [&] ( tbb::blocked_range<size_t> r, float local_sum ) {
            for ( size_t i = r.begin(); i < r.end(); ++ i ) {
                local_sum += std::sin(i);
            }
            return local_sum;
        }, std::plus<float>()
        );
    std::cout << " sum = " << sum << std::endl;

    auto t1 = std::chrono::steady_clock::now();
    auto dt = t1 - t0;
    int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
    std::cout << "time elapsed: " << ms << " ms" << std::endl;
#endif

    return 0;
}