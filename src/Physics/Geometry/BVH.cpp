#include "NexDynIPC/Physics/Geometry/BVH.h"

namespace NexDynIPC::Physics::Geometry {

//==============================================================================
// BVH Implementation
//==============================================================================

void BVH::init(const std::vector<std::array<Eigen::Vector3d, 2>>& aabbs)
{
    std::vector<AABB> aabbList;
    aabbList.reserve(aabbs.size());
    for (const auto& a : aabbs) {
        aabbList.emplace_back(a[0], a[1]);
    }
    init(aabbList);
}

void BVH::init(const std::vector<AABB>& aabbs)
{
    clear();

    if (aabbs.empty()) {
        return;
    }

    std::vector<int> primitives(aabbs.size());
    for (size_t i = 0; i < aabbs.size(); ++i) {
        primitives[i] = static_cast<int>(i);
    }

    nodes_.reserve(2 * aabbs.size() - 1);
    root_ = buildRecursive(primitives, aabbs);
}

void BVH::clear()
{
    nodes_.clear();
    root_ = -1;
}

bool BVH::empty() const
{
    return nodes_.empty() || root_ < 0;
}

int BVH::nodeCount() const
{
    return static_cast<int>(nodes_.size());
}

void BVH::intersectBox(const AABB& aabb, std::vector<unsigned int>& result) const
{
    intersectBox(aabb.min(), aabb.max(), result);
}

void BVH::intersectBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max,
    std::vector<unsigned int>& result) const
{
    result.clear();

    if (root_ < 0) {
        return;
    }

    AABB queryBBox(min, max);

    std::stack<int> stack;
    stack.push(root_);

    while (!stack.empty()) {
        int nodeId = stack.top();
        stack.pop();

        const Node& node = nodes_[nodeId];

        if (!node.bbox.intersects(queryBBox)) {
            continue;
        }

        if (node.isLeaf()) {
            result.push_back(static_cast<unsigned int>(node.primitive));
        } else {
            if (node.left >= 0)
                stack.push(node.left);
            if (node.right >= 0)
                stack.push(node.right);
        }
    }
}

void BVH::intersectPoint(const Eigen::Vector3d& point,
    std::vector<unsigned int>& result) const
{
    result.clear();

    if (root_ < 0) {
        return;
    }

    std::stack<int> stack;
    stack.push(root_);

    while (!stack.empty()) {
        int nodeId = stack.top();
        stack.pop();

        const Node& node = nodes_[nodeId];

        if (!node.bbox.contains(point)) {
            continue;
        }

        if (node.isLeaf()) {
            result.push_back(static_cast<unsigned int>(node.primitive));
        } else {
            if (node.left >= 0)
                stack.push(node.left);
            if (node.right >= 0)
                stack.push(node.right);
        }
    }
}

void BVH::intersectBVH(const BVH& other,
    std::vector<std::pair<int, int>>& result) const
{
    result.clear();

    if (root_ < 0 || other.root_ < 0) {
        return;
    }

    std::stack<std::pair<int, int>> stack;
    stack.emplace(root_, other.root_);

    while (!stack.empty()) {
        auto [nodeA, nodeB] = stack.top();
        stack.pop();

        const Node& a = nodes_[nodeA];
        const Node& b = other.nodes_[nodeB];

        if (!a.bbox.intersects(b.bbox)) {
            continue;
        }

        if (a.isLeaf() && b.isLeaf()) {
            result.emplace_back(a.primitive, b.primitive);
        } else if (a.isLeaf()) {
            if (b.left >= 0)
                stack.emplace(nodeA, b.left);
            if (b.right >= 0)
                stack.emplace(nodeA, b.right);
        } else if (b.isLeaf()) {
            if (a.left >= 0)
                stack.emplace(a.left, nodeB);
            if (a.right >= 0)
                stack.emplace(a.right, nodeB);
        } else {
            if (a.left >= 0 && b.left >= 0)
                stack.emplace(a.left, b.left);
            if (a.left >= 0 && b.right >= 0)
                stack.emplace(a.left, b.right);
            if (a.right >= 0 && b.left >= 0)
                stack.emplace(a.right, b.left);
            if (a.right >= 0 && b.right >= 0)
                stack.emplace(a.right, b.right);
        }
    }
}

int BVH::height() const
{
    return heightRecursive(root_);
}

int BVH::heightRecursive(int nodeId) const
{
    if (nodeId < 0 || nodeId >= static_cast<int>(nodes_.size())) {
        return 0;
    }

    const Node& node = nodes_[nodeId];

    if (node.isLeaf()) {
        return 1;
    }

    int leftHeight = heightRecursive(node.left);
    int rightHeight = heightRecursive(node.right);

    return 1 + std::max(leftHeight, rightHeight);
}

int BVH::leafCount() const
{
    int count = 0;
    for (const auto& node : nodes_) {
        if (node.isLeaf()) {
            ++count;
        }
    }
    return count;
}

int BVH::internalCount() const
{
    int count = 0;
    for (const auto& node : nodes_) {
        if (node.isInternal()) {
            ++count;
        }
    }
    return count;
}

int BVH::buildRecursive(std::vector<int>& primitives, const std::vector<AABB>& aabbs)
{
    if (primitives.empty()) {
        return -1;
    }

    int nodeId = static_cast<int>(nodes_.size());
    nodes_.push_back(Node());

    if (primitives.size() == 1) {
        nodes_[nodeId].bbox = aabbs[primitives[0]];
        nodes_[nodeId].primitive = primitives[0];
        return nodeId;
    }

    AABB centroidBBox;
    for (int prim : primitives) {
        centroidBBox.expand(aabbs[prim].center());
    }

    int axis = centroidBBox.maxExtentAxis();

    if (centroidBBox.extent()[axis] < 1e-10) {
        int mid = static_cast<int>(primitives.size()) / 2;
        std::vector<int> leftPrims(primitives.begin(), primitives.begin() + mid);
        std::vector<int> rightPrims(primitives.begin() + mid, primitives.end());

        int leftId = buildRecursive(leftPrims, aabbs);
        int rightId = buildRecursive(rightPrims, aabbs);

        nodes_[nodeId].left = leftId;
        nodes_[nodeId].right = rightId;
        nodes_[nodeId].bbox = nodes_[leftId].bbox.merged(nodes_[rightId].bbox);

        if (leftId >= 0)
            nodes_[leftId].parent = nodeId;
        if (rightId >= 0)
            nodes_[rightId].parent = nodeId;

        return nodeId;
    }

    double splitPos;
    int mid = findSAHSplit(primitives, aabbs, axis, splitPos);

    if (mid <= 0 || mid >= static_cast<int>(primitives.size())) {
        mid = static_cast<int>(primitives.size()) / 2;
        std::nth_element(primitives.begin(), primitives.begin() + mid,
            primitives.end(), [&aabbs, axis](int a, int b) {
                return aabbs[a].center()[axis] < aabbs[b].center()[axis];
            });
    }

    std::vector<int> leftPrims(primitives.begin(), primitives.begin() + mid);
    std::vector<int> rightPrims(primitives.begin() + mid, primitives.end());

    int leftId = buildRecursive(leftPrims, aabbs);
    int rightId = buildRecursive(rightPrims, aabbs);

    nodes_[nodeId].left = leftId;
    nodes_[nodeId].right = rightId;
    nodes_[nodeId].bbox = nodes_[leftId].bbox.merged(nodes_[rightId].bbox);

    if (leftId >= 0)
        nodes_[leftId].parent = nodeId;
    if (rightId >= 0)
        nodes_[rightId].parent = nodeId;

    return nodeId;
}

int BVH::findSAHSplit(const std::vector<int>& primitives,
    const std::vector<AABB>& aabbs, int axis, double& splitPos)
{
    const int numBins = 16;
    const int n = static_cast<int>(primitives.size());

    double minCoord = std::numeric_limits<double>::max();
    double maxCoord = std::numeric_limits<double>::lowest();

    for (int prim : primitives) {
        double c = aabbs[prim].center()[axis];
        minCoord = std::min(minCoord, c);
        maxCoord = std::max(maxCoord, c);
    }

    if (maxCoord - minCoord < 1e-10) {
        splitPos = minCoord;
        return n / 2;
    }

    std::vector<AABB> binAABBs(numBins);
    std::vector<int> binCounts(numBins, 0);

    double binWidth = (maxCoord - minCoord) / numBins;

    for (int prim : primitives) {
        double c = aabbs[prim].center()[axis];
        int bin = std::min(numBins - 1, static_cast<int>((c - minCoord) / binWidth));
        binAABBs[bin].expand(aabbs[prim]);
        binCounts[bin]++;
    }

    std::vector<double> leftArea(numBins, 0);
    std::vector<int> leftCount(numBins, 0);
    std::vector<double> rightArea(numBins, 0);
    std::vector<int> rightCount(numBins, 0);

    AABB leftBBox;
    int leftTotal = 0;
    for (int i = 0; i < numBins; ++i) {
        leftBBox.expand(binAABBs[i]);
        leftTotal += binCounts[i];
        leftArea[i] = leftBBox.surfaceArea();
        leftCount[i] = leftTotal;
    }

    AABB rightBBox;
    int rightTotal = 0;
    for (int i = numBins - 1; i >= 0; --i) {
        rightBBox.expand(binAABBs[i]);
        rightTotal += binCounts[i];
        rightArea[i] = rightBBox.surfaceArea();
        rightCount[i] = rightTotal;
    }

    double bestCost = std::numeric_limits<double>::max();
    int bestSplit = n / 2;
    splitPos = minCoord + (maxCoord - minCoord) / 2;

    for (int i = 0; i < numBins - 1; ++i) {
        if (leftCount[i] == 0 || rightCount[i + 1] == 0)
            continue;

        double cost = leftArea[i] * leftCount[i] + rightArea[i + 1] * rightCount[i + 1];

        if (cost < bestCost) {
            bestCost = cost;
            bestSplit = leftCount[i];
            splitPos = minCoord + (i + 1) * binWidth;
        }
    }

    return bestSplit;
}

} // namespace NexDynIPC::Physics::Geometry
