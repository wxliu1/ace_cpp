// test various cast here.

#include <iostream>

int main(int argc, char* argv[])
{
    double value1 = static_cast<double>(10/3);
    double value2 = static_cast<double>(10.0/3);
    double value3 = static_cast<double>(10/(3 * 1.0));
    double value4 = static_cast<int>(10/(3 * 1.0));
    int number = 10;
    double average = static_cast<double>(number) / 3;
    std::cout << "value1=" << value1 << std::endl;
    std::cout << "value2=" << value2 << std::endl;
    std::cout << "value3=" << value3 << std::endl;
    std::cout << "value4=" << value4 << std::endl;
    std::cout << "average=" << average << std::endl;
}