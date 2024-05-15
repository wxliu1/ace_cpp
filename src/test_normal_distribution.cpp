
// normal_distribution
#include <iostream>
#include <string>
#include <random>

int main()
{
  const int nrolls=10000;  // number of experiments
  const int nstars=100;    // maximum number of stars to distribute

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(5.0,2.0);

  constexpr int COUNT = 10;//20;
//   int p[10]={};
  int p[COUNT]={};

  for (int i=0; i<nrolls; ++i) {
    // 做一万次实验，产生一万个随机数  
    double number = distribution(generator);
    // 统计落在区间[0, 10)里面的，从0到9的每个整数附近的个数
    // if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
    if ((number>=0.0)&&(number<COUNT)) ++p[int(number)];
  }

  std::cout << "normal_distribution (5.0,2.0):" << std::endl;

//   for (int i=0; i<10; ++i) {
  for (int i=0; i<COUNT; ++i) {
    std::cout << i << "-" << (i+1) << ": ";
    // 输出star个数，个数由随机数在整数i附近出现的次数来决定
    std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
  }

  /// 结论：发现产生的随机数，大概率情况下是在期望值5.0附近，出现的可能性最大

  return 0;
}