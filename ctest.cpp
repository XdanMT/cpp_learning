#include <iostream>
#include <queue>
#include <vector>
#include <functional> // 包含 std::greater

int main() {
    // 使用 std::greater<int> 创建一个最小堆
	// less表示新插入的元素（在堆尾）个每个父节点比较，如果小（less），则一路往上跑，所以堆顶永远是最小的
	// greater表示新插入的元素（在堆尾）个每个父节点比较，如果大（greater），则一路往上跑，所以堆顶永远是最大的
    std::priority_queue<int, std::vector<int>, std::less<int>> pq;

    // 插入元素
    pq.push(10);
    pq.push(30);
    pq.push(20);
    pq.push(5);
    pq.push(1);

    // 输出并移除优先级最高的元素（最小元素），直到队列为空
    std::cout << "Priority Queue (min-heap): ";
    while (!pq.empty()) {
        std::cout << pq.top() << " "; // 输出当前堆顶元素
        pq.pop(); // 移除当前堆顶元素
    }
    std::cout << std::endl;

    return 0;
}
