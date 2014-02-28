

#include <stdio.h>
#include <map>
#include <set>
#include <algorithm>

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
Position movedBy(const Position p, const int dx, const int dy) {
    return Position(p.x + dx, p.y + dy);
}

Position invalidPosition() { return Position(-1, -1); }

struct NodeData {
    int actualCost;
    int estimatedCost;
    Position cameFrom;

    NodeData(int actualCost, int heuristicCost, Position cameFrom)
            : actualCost(actualCost), estimatedCost(actualCost + heuristicCost),
              cameFrom(cameFrom) { }
    NodeData() : actualCost(0), estimatedCost(0), cameFrom(0,0) {}
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

    std::map<Position, NodeData> open_set;
    std::map<Position, NodeData> closed_set;

    Position start;
    Position target;

    int getIndex( const Position pos ) {
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
        return sqrt(2) * std::min(dx, dy) + std::max(dx,dy) - std::min(dx,dy);
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
     * gets forced neighbors for position travelling in incomingX, incomingY
     * assumes 2 spots free in outNeighbors
     */
    int getForcedNeighbors(const Position pos, Position outNeighbors[2], int incomingX, int incomingY);

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
                } else if (open_set.find(p) != open_set.end()) {
                    printf("x");
                } else if (closed_set.find(p) != closed_set.end()) {
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
    open_set[start] = NodeData(0, heuristicCostEstimate(start), invalidPosition());

    while (!open_set.empty()) {
        //printf("======================================================\n");
        //printState();
        auto current = open_set.begin();
        auto pair = current;
        for (++pair; pair != open_set.end(); ++pair) {
            //printf("%d %d (%d) vs %d %d (%d) ",
            //        current->first.x, current->first.y, current->second.estimatedCost,
            //        pair->first.x, pair->first.y, pair->second.estimatedCost
            //      );
            if (pair->second.estimatedCost < current->second.estimatedCost) {
                //printf("re-placin!\n");
                current = pair;
            }
            //printf("\n");
        }

        Position currentPos = current->first;
        NodeData currentNodeData = current->second;

        if (currentPos == target) {
            // do stuff
            int cost = currentNodeData.actualCost;
            int i = cost;
            Position cameFrom = currentNodeData.cameFrom;
            while (!(currentPos == start)) {
                Position walker = currentPos;
 
                int dx = -sgn(currentPos.x - cameFrom.x);
                int dy = -sgn(currentPos.y - cameFrom.y);

                while (!(walker == cameFrom)) {
                    pOutBuffer[--i] = getIndex(walker);

                    if (dx != 0 && dy != 0) { // let's not walk through a wall
                        if (valid(movedBy(walker, dx, 0))) {
                            walker = movedBy(walker, dx, 0);
                            pOutBuffer[--i] = getIndex(walker); // another write here
                            walker = movedBy(walker, 0, dy);
                        } else {
                            walker = movedBy(walker, 0, dy);
                            pOutBuffer[--i] = getIndex(walker); // or here
                            walker = movedBy(walker, dx, 0);
                        }
                    } else {
                        walker = movedBy(walker, dx, dy);
                    }
                }

                currentPos = cameFrom;
                cameFrom = closed_set[cameFrom].cameFrom;
            }

            return cost;
        }

        open_set.erase(currentPos);
        closed_set[currentPos] = currentNodeData;

        Position successors[8];

        int num_successors = getSuccessors(currentPos, successors, currentNodeData.cameFrom);

        for (int i = 0; i < num_successors; i += 1) {

            Position successor = successors[i];

            //printf("successor: %d %d\n", successor.x, successor.y);

            if (closed_set.find(successor) != closed_set.end()) {
                continue;
            }


            int cost = currentNodeData.actualCost + distance(currentPos, successor);

            //printf("cost = %d\n", cost);

            float heuristic = heuristicCostEstimate(successor);

            if (open_set.find(successor) == open_set.end()
                    || cost + heuristic < open_set[successor].estimatedCost) {
                open_set[successor] = NodeData(cost, heuristic, currentPos);
            }
        }

    }
    return -1;
}

int JumpPointSearch::getSuccessors(const Position pos, Position outSuccessors[8], const Position cameFrom) {
    Position neighbors[8];
    getNeighbors(pos, neighbors, cameFrom);

    int successors_found = 0;

    for (int i = 0; i < 8; i += 1) {
        Position neighbor = neighbors[i];
        if (!valid(neighbor)) { continue; }

        int dx = sgn(neighbor.x - pos.x);
        int dy = sgn(neighbor.y - pos.y);

        Position result = jump(pos, dx, dy);
        if (valid(result)) {
            outSuccessors[successors_found] = result;
            successors_found += 1;
        }
    }
    return successors_found;
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

    int incomingX = sgn(pos.x - cameFrom.x);
    int incomingY = sgn(pos.y - cameFrom.y);

    int num_neighbors = 0;

    if (incomingX != 0 && incomingY == 0) { // moving on x only
        outNeighbors[num_neighbors++] = movedBy(pos, incomingX, 0);
    } else if (incomingX == 0 && incomingY != 0) { // moving on y only
        outNeighbors[num_neighbors++] = movedBy(pos, 0, incomingY);
    } else { // diagonal
        if (valid(movedBy(pos, incomingX, 0)) || valid(movedBy(pos, 0, incomingY))) { // cannot move across diagonals
            outNeighbors[num_neighbors++] = movedBy(pos, incomingX, 0);
            outNeighbors[num_neighbors++] = movedBy(pos, 0, incomingY);
            outNeighbors[num_neighbors++] = movedBy(pos, incomingX, incomingY);
        }
    }
    int num_forced_neighbors = getForcedNeighbors(pos, outNeighbors+num_neighbors, incomingX, incomingY);
    return num_neighbors + num_forced_neighbors;
}

int JumpPointSearch::getForcedNeighbors(const Position pos, Position outNeighbors[2], int incomingX, int incomingY) {

    int num_forced_neighbors = 0;
    if (incomingX != 0 && incomingY == 0 && valid(movedBy(pos, incomingX, 0))) {
        if (!valid(northOf(pos))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, incomingX, -1);
        }
        if (!valid(southOf(pos))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, incomingX, 1);
        }
    } else if (incomingX == 0 && incomingY != 0 && valid(movedBy(pos, 0, incomingY))) { 
        if (!valid(westOf(pos))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, -1, incomingY);
        }
        if (!valid(eastOf(pos))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, 1, incomingY);
        }
    } else { // diagonal
        if (!valid(movedBy(pos, -incomingX, 0)) && valid(movedBy(pos, 0, incomingY))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, -incomingX, incomingY);
        }
        if (!valid(movedBy(pos, 0, -incomingY)) && valid(movedBy(pos, incomingX, 0))) {
            outNeighbors[num_forced_neighbors++] = movedBy(pos, incomingX, -incomingY);
        }
    }
    return num_forced_neighbors;
}



Position JumpPointSearch::jump(const Position from, const int dx, const int dy) {
    Position forcedNeighborsBuffer[2];

    Position p = movedBy(from, dx, dy);

    //printf("Jumping from (%d, %d) in direction (%d, %d)\n", from.x, from.y, dx, dy);
    while (valid(p)) {
        if (!valid(movedBy(p, -dx, 0)) && !valid(movedBy(p, 0, -dy))) {
            // we moved across a diagonal wall
            return invalidPosition();
        }

        forcedNeighborsBuffer[0] = invalidPosition();
        forcedNeighborsBuffer[1] = invalidPosition();
        if (getForcedNeighbors(p, forcedNeighborsBuffer, dx, dy)) {
            if (valid(forcedNeighborsBuffer[0]) || valid(forcedNeighborsBuffer[1])) {
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


unsigned int* debugArray;


void makeBMP(int, int);


int main() {

    //const int nMapWidth = 13, nMapHeight = 13;

    //unsigned char pMap[] = {
    //    1,1,1,1,1, 1,1,1,1,1, 1,1,1,
    //    1,1,1,1,1, 1,1,0,0,0, 0,0,1,
    //    1,1,1,1,1, 1,1,1,1,1, 1,1,1,
    //    1,1,1,1,1, 0,1,0,1,1, 1,1,1,
    //    1,1,1,1,1, 0,1,0,0,0, 0,1,1,

    //    1,1,1,1,1, 0,1,0,1,1, 0,1,1,
    //    1,1,1,1,1, 0,1,0,1,1, 1,1,1,
    //    1,1,1,1,1, 0,1,0,1,1, 0,1,1,
    //    1,1,1,1,1, 0,1,0,0,0, 0,0,0,
    //    1,1,1,1,1, 0,1,1,1,1, 1,1,1,
    //    
    //    1,1,1,1,1, 0,1,1,1,1, 1,1,1,
    //    1,1,1,1,1, 0,0,0,0,1, 1,1,1,
    //    1,1,1,1,1, 0,1,1,1,1, 1,1,1
    //};
    

    const int nMapWidth = 1024, nMapHeight = 1024;


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

    debugArray = new unsigned int[nMapWidth * nMapHeight];

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
        if (pMap[outputBuffer[i]] == 0) {
            printf("%d\n", i);
            throw 4;
        }
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
