
#include <map>
#include <set>
#include <algorithm>

namespace { // wrap everything but FindPath in anonymous namespace

int sgn(int val) {
    return (val > 0) - (val < 0);
}

struct Position {
    int x;
    int y;

    Position(int x, int y) : x(x), y(y) {}
    Position() : x(-1), y(-1) {}

    bool operator==(const Position other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position other) const {
        return !(*this == other);
    }
    bool operator<(const Position other) const {
        if (x == other.x) { return y < other.y; }
        return x < other.x;
    }
};

Position southOf(const Position p) { return Position(p.x, p.y + 1); }
Position northOf(const Position p) { return Position(p.x, p.y - 1); }
Position westOf(const Position p) { return Position(p.x - 1, p.y); }
Position eastOf(const Position p) { return Position(p.x + 1, p.y); }
Position movedBy(const Position p, const int dx, const int dy) {
    return Position(p.x + dx, p.y + dy);
}

Position invalidPosition() { return Position(-1, -1); }

struct NodeData {
    int actualCost;
    float estimatedCost;
    Position cameFrom;

    NodeData(int actualCost, float heuristicCost, Position cameFrom)
            : actualCost(actualCost), estimatedCost(actualCost + heuristicCost),
              cameFrom(cameFrom) { }
    NodeData() : actualCost(0), estimatedCost(0), cameFrom(0,0) {}
};


/**
 * Class wrapper for implementation of Jump Point Search.
 *
 * Wrapped in a class to reduce passing around of variables such as map width.
 * 
 * Used by the function FindPath.
 */
class JumpPointSearch {
    const unsigned char* pMap;
    const int nMapWidth;
    const int nMapHeight;
    int* pOutBuffer;
    const int nOutBufferSize;

    std::map<Position, NodeData> openSet;
    std::map<Position, NodeData> closedSet;

    Position start;
    Position target;

    int getIndex(const Position pos) {
        return pos.y * nMapWidth + pos.x;
    }
    int distance(const Position pos1, const Position pos2) {
        // manhattan for actual distance
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
    }
    float heuristicCostEstimate(const Position pos1) {
        // chebyshev for the heuristic
        int dx = abs(pos1.x - target.x);
        int dy = abs(pos1.y - target.y);
        return sqrt(2.0f) * std::min(dx, dy) + std::max(dx,dy) - std::min(dx,dy);
    }
    bool withinBounds(const Position pos) {
        return 0 <= pos.x && pos.x < nMapWidth
            && 0 <= pos.y && pos.y < nMapHeight;
    }
    bool valid(const Position pos) {
        return withinBounds(pos) && pMap[getIndex(pos)] == 1;
    }
    
    /**
     * Reconstructs path from currentPos (which is going to be == target)
     * Returns length of said path.
     */
    int JumpPointSearch::reconstructPath(Position currentPos, NodeData currentNodeData);

    /**
     * Traverses the grid in specified direction.
     *
     * Returns a position which needs to be visited,
     * or invalidPosition() if no such node was found.
     */
    Position jump(const Position from, const int dx, const int dy);

    /**
     * Gets successors for a position
     * returns number of successors written in outSuccessors
     * assumes outNeighbors to be of length 8 or more
     */
    int getSuccessors(const Position pos, Position outSuccessors[8], const Position cameFrom);
    /**
     * Gets successors for a position
     * returns number of neighbors written in outNeighbors
     * assumes outNeighbors to be of length 8 or more
     * positions in outNeighbors may be invalid
     */
    int getNeighbors(const Position pos, Position outNeighbors[8], const Position cameFrom);

    /**
     * Gets forced neighbors for position travelling in dx, dy.
     * Assumes 2 spots free in outNeighbors.
     * Returns number of forced neighbors found.
     */
    int getForcedNeighbors(const Position pos, Position outNeighbors[2], int dx, int dy);

public:
    /**
     * Constructs the wrapper with the arguments for the path finding.
     */
    JumpPointSearch( const int nStartX, const int nStartY,
            const int nTargetX, const int nTargetY,
            const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
            int* pOutBuffer, const int nOutBufferSize )
        : start(nStartX, nStartY), target(nTargetX, nTargetY),
          pMap(pMap), nMapWidth(nMapWidth), nMapHeight(nMapHeight),
          pOutBuffer(pOutBuffer), nOutBufferSize(nOutBufferSize) {}

    /**
     * Entry point function.
     * Returns length of path found. Path written to pOutBuffer.
     */
    int findPath();
};

int JumpPointSearch::reconstructPath(Position currentPos, NodeData currentNodeData) {
    int cost = currentNodeData.actualCost;

    int i = cost;
    Position cameFrom = currentNodeData.cameFrom;
    while (currentPos != start) {
        Position walker = currentPos;
        int dx = -sgn(currentPos.x - cameFrom.x);
        int dy = -sgn(currentPos.y - cameFrom.y);

        while (walker != cameFrom) {
            pOutBuffer[--i] = getIndex(walker);

            if (dx != 0 && dy != 0) { // let's not walk through a wall
                if (valid(movedBy(walker, dx, 0))) {
                    walker = movedBy(walker, dx, 0);
                    pOutBuffer[--i] = getIndex(walker); // another write here
                    walker = movedBy(walker, 0, dy);
                }
                else {
                    walker = movedBy(walker, 0, dy);
                    pOutBuffer[--i] = getIndex(walker); // or here
                    walker = movedBy(walker, dx, 0);
                }
            }
            else {
                walker = movedBy(walker, dx, dy);
            }
        }
        currentPos = cameFrom;
        cameFrom = closedSet[cameFrom].cameFrom;
    }
    return cost;
}

int JumpPointSearch::findPath() {
    openSet[start] = NodeData(0, heuristicCostEstimate(start), invalidPosition());

    while (!openSet.empty()) {
        auto current = openSet.begin();
        auto pair = current;
        for (++pair; pair != openSet.end(); ++pair) {
            if (pair->second.estimatedCost < current->second.estimatedCost) {
                current = pair;
            }
        }
        Position currentPos = current->first;
        NodeData currentNodeData = current->second;

        if (currentPos == target) {
            return reconstructPath(currentPos, currentNodeData);
        }
        openSet.erase(currentPos);
        closedSet[currentPos] = currentNodeData;

        Position successors[8];
        int numSuccessors = getSuccessors(currentPos, successors, currentNodeData.cameFrom);

        for (int i = 0; i < numSuccessors; i += 1) {
            Position successor = successors[i];

            if (closedSet.find(successor) != closedSet.end()) {
                continue;
            }
            int cost = currentNodeData.actualCost + distance(currentPos, successor);

            if (cost > nOutBufferSize) { // dont bother adding nodes that are unreachable
                continue;
            }
            float heuristic = heuristicCostEstimate(successor);

            if (openSet.find(successor) == openSet.end()
                    || cost + heuristic < openSet[successor].estimatedCost) {
                openSet[successor] = NodeData(cost, heuristic, currentPos);
            }
        }
    }
    return -1;
}

int JumpPointSearch::getSuccessors(const Position pos, Position outSuccessors[8], const Position cameFrom) {
    Position neighbors[8];
    getNeighbors(pos, neighbors, cameFrom);

    int successorsFound = 0;

    for (int i = 0; i < 8; i += 1) {
        Position neighbor = neighbors[i];
        if (!valid(neighbor)) { continue; }

        int dx = sgn(neighbor.x - pos.x);
        int dy = sgn(neighbor.y - pos.y);

        Position result = jump(pos, dx, dy);
        if (valid(result)) {
            outSuccessors[successorsFound] = result;
            successorsFound += 1;
        }
    }
    return successorsFound;
}

int JumpPointSearch::getNeighbors(const Position pos, Position outNeighbors[8], const Position cameFrom) {
    if (!valid(cameFrom)) { // start position, return everything
        outNeighbors[0] = westOf(northOf(pos));
        outNeighbors[1] = northOf(pos);
        outNeighbors[2] = eastOf(northOf(pos));
        outNeighbors[3] = westOf(pos);
        outNeighbors[4] = eastOf(pos);
        outNeighbors[5] = westOf(southOf(pos));
        outNeighbors[6] = southOf(pos);
        outNeighbors[7] = eastOf(southOf(pos));
        return 8;
    }

    int dx = sgn(pos.x - cameFrom.x);
    int dy = sgn(pos.y - cameFrom.y);

    int numNeighbors = 0;

    if (dx != 0 && dy == 0) { // moving on x only
        outNeighbors[numNeighbors++] = movedBy(pos, dx, 0);
    } else if (dx == 0 && dy != 0) { // moving on y only
        outNeighbors[numNeighbors++] = movedBy(pos, 0, dy);
    } else { // diagonal
        if (valid(movedBy(pos, dx, 0)) || valid(movedBy(pos, 0, dy))) { // cannot move across diagonal walls
            outNeighbors[numNeighbors++] = movedBy(pos, dx, 0);
            outNeighbors[numNeighbors++] = movedBy(pos, 0, dy);
            outNeighbors[numNeighbors++] = movedBy(pos, dx, dy);
        }
    }
    int numForcedNeighbors = getForcedNeighbors(pos, outNeighbors+numNeighbors, dx, dy);
    return numNeighbors + numForcedNeighbors;
}

int JumpPointSearch::getForcedNeighbors(const Position pos, Position outNeighbors[2], int dx, int dy) {

    int numForcedNeighbors = 0;
    if (dx != 0 && dy == 0 && valid(movedBy(pos, dx, 0))) {
        if (!valid(northOf(pos))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, dx, -1);
        }
        if (!valid(southOf(pos))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, dx, 1);
        }
    } else if (dx == 0 && dy != 0 && valid(movedBy(pos, 0, dy))) { 
        if (!valid(westOf(pos))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, -1, dy);
        }
        if (!valid(eastOf(pos))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, 1, dy);
        }
    } else { // diagonal
        if (!valid(movedBy(pos, -dx, 0)) && valid(movedBy(pos, 0, dy))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, -dx, dy);
        }
        if (!valid(movedBy(pos, 0, -dy)) && valid(movedBy(pos, dx, 0))) {
            outNeighbors[numForcedNeighbors++] = movedBy(pos, dx, -dy);
        }
    }
    return numForcedNeighbors;
}

Position JumpPointSearch::jump(const Position from, const int dx, const int dy) {

    Position p = movedBy(from, dx, dy);

    while (valid(p)) {
        if (!valid(movedBy(p, -dx, 0)) && !valid(movedBy(p, 0, -dy))) {
            // we moved across a diagonal wall
            return invalidPosition();
        }
        
        Position forcedNeighborsBuffer[2];
        int numForcedNeighbors = getForcedNeighbors(p, forcedNeighborsBuffer, dx, dy);
        for (int i = 0; i < numForcedNeighbors; i += 1) {
            if (valid(forcedNeighborsBuffer[i])) {
                return p;
            }
        }
        if (p == target) {
            return p;
        }

        if (dx != 0 && dy != 0) {
            Position recursivejump1 = jump(p, dx, 0);
            if (valid(recursivejump1)) { return p; }
            Position recursivejump2 = jump(p, 0, dy);
            if (valid(recursivejump2)) { return p; }
        }

        p = movedBy(p, dx, dy);
    }

    return invalidPosition();
}

}

/**
 * Finds the path from (nStartX, nStartY) to (nTargetX, nTargetY)
 * through the grid pMap with dimensions (nMapWidth, nMapHeight)
 * where traversable locations are marked with 1 and non-traversable as 0.
 *
 * pOutBuffer is filled with indexes into pMap.
 * 
 * If the optimal path is longer than nOutBufferSize, or no path can
 * be found, -1 is returned.
 */
int FindPath( const int nStartX, const int nStartY,
        const int nTargetX, const int nTargetY,
        const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
        int* pOutBuffer, const int nOutBufferSize ) {

    return JumpPointSearch(
            nStartX, nStartY,
            nTargetX, nTargetY,
            pMap, nMapWidth, nMapHeight,
            pOutBuffer, nOutBufferSize
        ).findPath();
}

