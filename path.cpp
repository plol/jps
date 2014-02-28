

#include <stdio.h>
#include <map>
#include <set>
#include <algorithm>
#include <queue>
#include <vector>
#include <functional>

#include <string.h>

namespace {

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
    bool operator<(const Position other) const {
        if (x == other.x) { return y < other.y; }
        return x < other.x;
    }
};

Position southOf(const Position p) { return Position(p.x, p.y + 1); }
Position northOf(const Position p) { return Position(p.x, p.y - 1); }
Position westOf(const Position p) { return Position(p.x - 1, p.y); }
Position eastOf(const Position p) { return Position(p.x + 1, p.y); }

Position invalidPosition() { return Position(-1, -1); }

struct NodeData {
    int cost;
    Position cameFrom;

    NodeData(int cost, Position cameFrom)
            : cost(cost), cameFrom(cameFrom) { }
    NodeData() : cost(0), cameFrom(0,0) {}
};

struct OpenSetData {
    int cost;
    Position pos;

    OpenSetData(int cost, Position pos) : cost(cost), pos(pos) { }

    bool operator>(const OpenSetData other) const {
        if (cost == other.cost) {
            return pos < other.pos;
        }
        return cost > other.cost;
    }
};


/**
 * Class wrapper for implementation of Jump Point Search.
 *
 * Wrapped in a class to reduce passing around of variables such as map width.
 *
 */
class JumpPointSearch {
    const unsigned char* pMap;
    const int nMapWidth;
    const int nMapHeight;
    int* pOutBuffer;
    const int nOutBufferSize;

    std::priority_queue<OpenSetData, std::vector<OpenSetData>, std::greater<OpenSetData>> open_set;
    std::set<Position> closed_set;

    std::map<Position, NodeData> nodeData;

    Position start;
    Position target;

    int getIndex( const Position pos ) {
        return pos.y * nMapWidth + pos.x;
    }

    bool withinBounds(const Position pos) {
        return 0 <= pos.x && pos.x < nMapWidth
            && 0 <= pos.y && pos.y < nMapHeight;
    }

    bool valid(const Position pos) {
        return withinBounds(pos) && pMap[getIndex(pos)] == 1;
    }

    /**
     * Traverses the grid in specified direction.
     *
     * Returns a position which needs to be added to the open set,
     * or invalidPosition() if no such node was found.
     */
    Position jump(const Position from, const int dx, const int dy);


    void addNodeData(Position p, int cost, Position cameFrom) {
        auto iterator = nodeData.find(p);

        if (iterator == nodeData.end()) {
            nodeData[p] = NodeData(cost, cameFrom);
        } else {
            if (iterator->second.cost > cost) {
                //printf("%d %d\n", iterator->first.x, iterator->first.y);
                printf("Found alternate path to node which was shorter :( %d vs %d\n", iterator->second.cost, cost);
                //throw 4;
                //nodeData[p] = NodeData(cost, cameFrom);
            }
        }
    }

    void addToProcessing(Position p, Position cameFrom) {
        if (!valid(p)) {
            return;
        }

        int cost = nodeData[cameFrom].cost;
        cost += abs(p.x-cameFrom.x) + abs(p.y - cameFrom.y);

        addNodeData(p, cost, cameFrom);

        if (closed_set.find(p) == closed_set.end()) {
            open_set.push(OpenSetData(cost, p));
        }
    }
public:

    JumpPointSearch( const int nStartX, const int nStartY,
            const int nTargetX, const int nTargetY,
            const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
            int* pOutBuffer, const int nOutBufferSize )
        : start(nStartX, nStartY), target(nTargetX, nTargetY),
          pMap(pMap), nMapWidth(nMapWidth), nMapHeight(nMapHeight),
          pOutBuffer(pOutBuffer), nOutBufferSize(nOutBufferSize) {}


    /**
     * Entry point function
     */
    int findPath();



    void printState() {
        printf("+");
        for (int x = 0; x < nMapWidth; x += 1) {
            printf("-");
        }
        printf("+\n");
        for (int y = 0; y < nMapHeight; y += 1) {
            printf("|");
            for (int x = 0; x < nMapWidth; x += 1) {


                Position p(x,y);

                if (p == start) {
                    printf("s");
                } else if (p == target) {
                    printf("t");
                } else if (nodeData.find(p) != nodeData.end()) {
                    printf(".");
                } else if (valid(p)) {
                    printf(" ");
                } else {
                    printf("#");
                }


            }
            printf("|\n");
        }
        printf("+");
        for (int x = 0; x < nMapWidth; x += 1) {
            printf("-");
        }
        printf("+\n");
    }


};

int JumpPointSearch::findPath() {
    closed_set.insert(start);

    nodeData[start] = NodeData(0, start);

    addToProcessing(jump(start, 1, 0), start);
    addToProcessing(jump(start, 0, -1), start);
    addToProcessing(jump(start, -1, 0), start);
    addToProcessing(jump(start, 0, 1), start);

    while (!open_set.empty()) {
        //printf("======================================================\n");
        //printState();
        auto current = open_set.top().pos;

        if (current == target) {
            // do stuff
            int cost = nodeData[target].cost;
            int i = cost;
            while (!(current == start)) {
                Position walker = current;
                Position cameFrom = nodeData[current].cameFrom;
 
                int dx = -sgn(current.x - cameFrom.x);
                int dy = -sgn(current.y - cameFrom.y);

                while (!(walker == cameFrom)) {
                    i -= 1;
                    pOutBuffer[i] = getIndex(walker);
                    walker.x += dx;
                    walker.y += dy;
                }

                current = cameFrom;
            }

            return cost;
        }

        open_set.pop();
        closed_set.insert(current);

        Position cameFrom = nodeData[current].cameFrom;

        int dx = sgn(current.x - cameFrom.x);
        int dy = sgn(current.y - cameFrom.y);


        if (dx != 0) {
            addToProcessing(jump(current, dx, 0), current);
            addToProcessing(jump(current, 0, 1), current);
            addToProcessing(jump(current, 0, -1), current);
        } else {
            addToProcessing(jump(current, 1, 0), current);
            addToProcessing(jump(current, -1, 0), current);
            addToProcessing(jump(current, 0, dy), current);
        }
    }

    return -1;
}

unsigned int* debugArray;


// expand sideways when going north or south
// only do secondary jumps east and west

Position JumpPointSearch::jump(const Position from, const int dx, const int dy) {

    std::function<Position (const Position)> leftOf, rightOf, forwardOf;

    //printf("%d %d ", from.x, from.y);
    if (dx == 1) {
        //printf("jump to east\n");

        leftOf = &northOf;
        rightOf = &southOf;
        forwardOf = &eastOf;
    } else if (dx == -1) {
        //printf("jump to west\n");

        leftOf = &southOf;
        rightOf = &northOf;
        forwardOf = &westOf;
    } else if (dy == 1) {
        //printf("jump to south\n");

        leftOf = &eastOf;
        rightOf = &westOf;
        forwardOf = &southOf;
    } else if (dy == -1) {
        //printf("jump to north\n");

        leftOf = &westOf;
        rightOf = &eastOf;
        forwardOf = &northOf;
    } else {
        printf("erraneous call to jump\n");
        throw 4;
    }

    bool previous_left_clear = valid(leftOf(from));
    bool previous_right_clear = valid(rightOf(from));

    Position p(forwardOf(from));
    //printf("ActualCost: %d\n", cost);
    //debugArray[getIndex(from)] = cost;

    while (valid(p)) {

        if (p == target) {
            return p;
        }

        bool left_clear = valid(leftOf(p));
        bool right_clear = valid(rightOf(p));

        if (dy != 0) {
            Position p1 = jump(p, -1, 0);
            if (valid(p1)) return p;
            Position p2 = jump(p, 1, 0);
            if (valid(p2)) return p;
        } else {
            if (left_clear && !previous_left_clear) {
                return p;
            }
            if (right_clear && !previous_right_clear) {
                return p;
            }
        }
        previous_left_clear = left_clear;
        previous_right_clear = right_clear;
        p = forwardOf(p);
    }
    return invalidPosition();
}

}


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




void makeBMP(int, int);


int main() {

    //const int nMapWidth = 5, nMapHeight = 5;

    //unsigned char pMap[] = {
    //    1,1,1,1,1,
    //    1,0,1,0,1,
    //    1,1,1,1,1,
    //    1,0,0,0,0,
    //    1,1,1,1,1
    //};
    //
    

    const int nMapWidth = 1024, nMapHeight = 1024;

    debugArray = new unsigned int[nMapWidth * nMapHeight];

    unsigned char* pMap = new unsigned char[nMapWidth * nMapHeight];

    for (int i = 0; i < nMapWidth * nMapHeight; i++) {
        pMap[i] = i % 6 != 0;
        if (i % 7 == 0) {
            pMap[i] = 0;
        }
    }
    for (int x = 0; x < nMapWidth; x++) {
        for (int y = 0; y < nMapHeight; y++) {
            if (x % 10 == 0 && y % 2 == 0) {
                pMap[x + y * nMapWidth] = 0;
            }
        }
    }

    pMap[0] = 1;

    for (int i = 0; i < nMapWidth*nMapHeight; i += 1) {
        if (pMap[i] == 0) {
            debugArray[i] = -1;
        } else {
            debugArray[i] = -2;
        }
    }
    debugArray[0] = -3;

    int* outputBuffer = new int[1024*1024];

    int pathLength = FindPath(0, 0, nMapWidth-1, nMapHeight-1, pMap, nMapWidth, nMapHeight, outputBuffer, 1024*1024);

    printf("i = %d\n", pathLength);
    for (int i = 0; i < pathLength; i += 1) {
        //printf("%d\n", outputBuffer[i]);
        debugArray[outputBuffer[i]] = -3;
    }
    debugArray[0] = -3;

    makeBMP(nMapWidth, nMapHeight);
}








void makeBMP(int mapWidth, int mapHeight)
{
    FILE *f;
    unsigned char *img = NULL;
    int filesize = 54 + 3 * mapWidth*mapHeight;  //w is your image width, h is image height, both int
    if (img)
        free(img);
    img = (unsigned char *)malloc(3 * mapWidth*mapHeight);
    memset(img, 0, sizeof(img));

    for (int x = 0; x < mapWidth; x++)
    {
        for (int y = 0; y < mapHeight; y++)
        {
            if (debugArray[x + y*mapWidth] == -1) {
                img[(x + y*mapWidth) * 3 + 2] = (unsigned char)(0);
                img[(x + y*mapWidth) * 3 + 1] = (unsigned char)(0);
                img[(x + y*mapWidth) * 3 + 0] = (unsigned char)(0);
            }
            else if (debugArray[x + y*mapWidth] == -2) {
                img[(x + y*mapWidth) * 3 + 2] = (unsigned char)(255);
                img[(x + y*mapWidth) * 3 + 1] = (unsigned char)(255);
                img[(x + y*mapWidth) * 3 + 0] = (unsigned char)(255);
            }
            else if (debugArray[x + y*mapWidth] == -3) {
                img[(x + y*mapWidth) * 3 + 2] = (unsigned char)(0);
                img[(x + y*mapWidth) * 3 + 1] = (unsigned char)(0);
                img[(x + y*mapWidth) * 3 + 0] = (unsigned char)(255);
            }
            else {
                img[(x + y*mapWidth) * 3 + 2] = (unsigned char)((float)debugArray[x + y*mapWidth] / 105334.f * 255.f);
                img[(x + y*mapWidth) * 3 + 1] = (unsigned char)(0);
                img[(x + y*mapWidth) * 3 + 0] = (unsigned char)(0);
            }

        }
    }

    unsigned char bmpfileheader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
    unsigned char bmpinfoheader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };
    unsigned char bmppad[3] = { 0, 0, 0 };

    bmpfileheader[2] = (unsigned char)(filesize);
    bmpfileheader[3] = (unsigned char)(filesize >> 8);
    bmpfileheader[4] = (unsigned char)(filesize >> 16);
    bmpfileheader[5] = (unsigned char)(filesize >> 24);

    bmpinfoheader[4] = (unsigned char)(mapWidth);
    bmpinfoheader[5] = (unsigned char)(mapWidth >> 8);
    bmpinfoheader[6] = (unsigned char)(mapWidth >> 16);
    bmpinfoheader[7] = (unsigned char)(mapWidth >> 24);
    bmpinfoheader[8] = (unsigned char)(mapHeight);
    bmpinfoheader[9] = (unsigned char)(mapHeight >> 8);
    bmpinfoheader[10] = (unsigned char)(mapHeight >> 16);
    bmpinfoheader[11] = (unsigned char)(mapHeight >> 24);

    f = fopen("img.bmp", "wb");
    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);
    for (int i = 0; i<mapHeight; i++)
    {
        fwrite(img + (mapWidth*(mapHeight - i - 1) * 3), 3, mapWidth, f);
        fwrite(bmppad, 1, (4 - (mapWidth * 3) % 4) % 4, f);
    }
    fclose(f);
}
