hello
#include <iostream>

int main() {
    long long a = 0, b = 1;
    std::cout << "前 10 个斐波那契数：";
    for (int i = 0; i < 10; ++i) {
        std::cout << a << ' ';
        long long t = a + b;
        a = b;
        b = t;
    }
    std::cout << "\n";

    int n = 0;
    std::cout << "输入一个整数（比如 5）：";
    if (std::cin >> n) {
        std::cout << "你输入的是 " << n << "，它的平方是 " << n * n << "\n";
    }
    return 0;
}

