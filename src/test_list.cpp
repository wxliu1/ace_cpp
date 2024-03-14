// test std::list

#include <list>
#include <algorithm>
#include <iostream>

// erase方法返回的是一个迭代器，指向被删除范围之后的第一个元素，可以用来继续迭代器的操作。如果删除了最后一个元素，返回的迭代器会指向list.end()
// erase()函数和remove_if()函数在删除元素后，容器的迭代器会失效，因此在循环中使用erase()时需要更新迭代器。而remove()函数则是直接在容器中移动元素，不会影响迭代器的有效性。

int main(char *argc, char **argv)
{
    /*
        // 根据迭代器删除单个元素
        std::list<int> myList = {1, 2, 3, 4, 5};
        auto it = std::find(myList.begin(), myList.end(), 3);
        if (it != myList.end())
            myList.erase(it); // 删除值为3的元素

        // 根据迭代器删除一个范围内的元素
        std::list<int> myList = {1, 2, 3, 4, 5};
        auto start_it = std::find(myList.begin(), myList.end(), 2);
        auto end_it = std::find(myList.begin(), myList.end(), 4);
        if (start_it != myList.end() && end_it != myList.end())
            myList.erase(start_it, end_it); // 删除值为2和3的元素

        // 删除特定值的所有元素
        std::list<int> myList = {1, 2, 3, 2, 1};
        myList.remove(2); // 删除值为2的所有元素
    */
    
    std::list<int> myList = {16, 1, 2, 3, 4, 5, 6, 27, 8, 9, 10, 11, 13, 14};

    // 使用remove_if()函数，该函数接受一个谓词作为参数，在list中找到所有满足谓词条件的元素并删除。例如，删除所有大于10的元素可以使用以下代码：
    myList.remove_if([](const int &value)
                   { return value > 10; });
/*
    for (auto it = myList.begin(); it != myList.end(); ++it)
    {
        if (*it == 8)
        {
            auto it2 = it;
            it++; // 或者使用 ++it 亦可。// Notice: 迭代器指向下一个位置，然后再erase, 迭代器才不会失效,然后--it, 最后for( ; ;++it),顺序就对上了。

            myList.erase(it2);
            --it;
            continue ;
        }
        
        std::cout << *it << " ";
    }

    std::cout << std::endl;
*/


    for (auto it = myList.begin(); it != myList.end(); ++it)
    {
        std::cout << *it << " ";
        // if(*it == 14) break ;
        // ++it;
    }

    std::cout << std::endl;


/*
    for (auto it = myList.begin(); it != myList.end();)
    {
        if (*it % 2 == 0)
        {
            it = myList.erase(it); // erase返回刪除元素的下一个元素的迭代器
        }
        else
        {
            ++it;
        }
    }
*/

    return 0;
}