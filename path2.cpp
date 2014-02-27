

#include <stdio.h>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>




struct Position {
    int x;
    int y;

    Position(int x, int y) : x(x), y(y) {}
    Position() : x(-1), y(-1) {}


    int getIndex( const int nMapWidth ) {
        return y * nMapWidth + x;
    }

    Position below() const { return Position(x, y + 1); }
    Position above() const { return Position(x, y - 1); }
    Position leftOf() const { return Position(x - 1, y); }
    Position rightOf() const { return Position(x + 1, y); }

    bool operator==(const Position other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Position other) const {
        if (x == other.x) { return y < other.y; }
        return x < other.x;
    }
};


int HeuristicCostEstimate( const Position pos1, const Position pos2 ) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}

bool WithinBounds( const Position pos,
        const int nMapWidth, const int nMapHeight ) {

    return 0 <= pos.x && pos.x < nMapWidth
        && 0 <= pos.y && pos.y < nMapHeight;
}



struct position_hash {
    size_t operator()(const Position& pos) const {
        long long x = pos.x << sizeof(int) | pos.y;
        return std::hash<long long>()(x);
    }
};

int FindPath( const int nStartX, const int nStartY,
        const int nTargetX, const int nTargetY,
        const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
        int* pOutBuffer, const int nOutBufferSize ) {

    static_assert(sizeof(int)*2 == sizeof(long long), "");


    std::set<Position> open_set, closed_set;

    std::map<Position, Position> came_from;

    std::map<Position, int> g_score;
    std::map<Position, int> f_score;
    //std::unordered_set<Position, position_hash> open_set, closed_set;

    //std::unordered_map<Position, Position, position_hash> came_from;

    //std::unordered_map<Position, int, position_hash> g_score;
    //std::unordered_map<Position, int, position_hash> f_score;


    Position start(nStartX, nStartY);
    Position target(nTargetX, nTargetY);

    open_set.insert(start);
    
    g_score[start] = 0;
    f_score[start] = HeuristicCostEstimate(start, target);


    while (!open_set.empty()) {

        Position current = *open_set.begin();

        for (Position alt : open_set) {
            if (f_score[alt] < f_score[current]) {
                current = alt;
            }
        }

        if (current == target) {
            int i = g_score[current]+0;
            int ret = i;
            if (i >= nOutBufferSize) {
                return -2;
            }

            while (i > 0) {
                i -= 1;
                pOutBuffer[i] = current.getIndex(nMapWidth);
                current = came_from[current];
            }
            return ret;
        }


        open_set.erase(current);

        closed_set.insert(current);

        Position neighbors[4] = {
            current.leftOf(),
            current.rightOf(),
            current.above(),
            current.below()
        };

        for (int i = 0; i < 4; i += 1) {
            Position neighbor = neighbors[i];

            if (!WithinBounds(neighbor, nMapWidth, nMapHeight) || pMap[neighbor.getIndex(nMapWidth)] == 0 ) {
                continue;
            }

            int new_g_score = g_score[current] + 1;
            int new_f_score = new_g_score + HeuristicCostEstimate(neighbor, target);

            if (closed_set.find(neighbor) != closed_set.end() && new_f_score >= f_score[neighbor]) {
                continue;
            }

            if (open_set.find(neighbor) == open_set.end() || new_f_score < f_score[neighbor]) {
                came_from[neighbor] = current;

                g_score[neighbor] = new_g_score;
                f_score[neighbor] = new_f_score;

                if (open_set.find(neighbor) == open_set.end()) {
                    open_set.insert(neighbor);
                }
            }
        }
    }
    return -1;
}






int main() {

    const int nMapWidth = 256, nMapHeight = 256;
    unsigned char* pMap = new unsigned char[nMapWidth * nMapHeight];

    for (int i = 0; i < nMapWidth * nMapHeight; i += 1) {
        pMap[i] = i % 6 != 0;
    }

    int* outputBuffer = new int[1024*1024];

    int pathLength = FindPath(0, 0, nMapWidth-1, nMapHeight-1, pMap, nMapWidth, nMapHeight, outputBuffer, 1024*1024);

    printf("i = %d\n", pathLength);
    for (int i = 0; i < pathLength; i += 1) {
        //printf("%d\n", outputBuffer[i]);
    }
}

