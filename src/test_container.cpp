
#include <iostream>
#include <vector>
#include <map>
#include <string>
using std::vector;
using std::map;
using std::string;

int main(int argc, char* argv[])
{
    // vector<int> vec = { 1, 2, 3, 4, 5, 6 };
    vector<int> vec = { 6, 5, 4, 3, 2, 1 };

    vec.emplace_back(7);
    vec.emplace(vec.end(), 8);
    vec.push_back(9);

    for(auto val : vec)
    {
        std::cout << val << ", ";
        std::cout << "a" << (val+1) << ", ";
    }
    std::cout << std::endl;
    
    vector<int>::iterator iter = vec.begin();
    for(; iter != vec.end(); )
    {
        std::cout << *iter << ", ";
        std::cout << "a" << *(iter+1) << ", ";
        iter++;
    }
    std::cout << std::endl;

    map<string, int> mmap;
    mmap["aaa"] = 2;
    mmap["bbb"] = 3;
    mmap["ccc"] = 1;

    for(const auto kv : mmap)
    {
        std::cout << kv.first << std::endl;
        std::cout << kv.second << std::endl;
    }
    
    map<string, int>::iterator iter2 = mmap.begin();
    for( ;iter2 != mmap.end(); iter2++)
    {
        std::cout << iter2->first << ", ";
        // auto iter3 = iter2 + 1; // error
        // std::cout << ":" << (iter2 + 1)->first << ", "; // error
    }

    return 0;
}