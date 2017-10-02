#include "2drobots.h"
using namespace std;
double activatefunc(double x){
    return tanh(x);
}
/**
    Direction Table (From me)
    -x : 0
    +x : 1
    -y : 2
    +y : 3
    -z : 4
    +z : 5
**/
void train(robot &r){
    for(int x=1;x<=Bsize;x++){
        for(int y=1;y<=Bsize;y++){
            for(int z=1;z<=Bsize;z++){
                r.brain[x][y][z] = activatefunc(
                   r.brain[x-1][y][z]*r.weight[x][y][z][0]+
                   r.brain[x+1][y][z]*r.weight[x][y][z][1]+
                   r.brain[x][y-1][z]*r.weight[x][y][z][2]+
                   r.brain[x][y+1][z]*r.weight[x][y][z][3]+
                   r.brain[x][y][z-1]*r.weight[x][y][z][4]+
                   r.brain[x][y][z+1]*r.weight[x][y][z][5]
                );
            }
        }
    }

    for(int x=1;x<=Bsize;x++){
        for(int y=1;y<=Bsize;y++){
            for(int z=1;z<=Bsize;z++){
                for(int w=0;w<numoutput;w++){
                    r.weight[x][y][z][w]=activatefunc(
                    r.brain[x-1][y][z]*neuronweight[0][w]+
                    r.brain[x+1][y][z]*neuronweight[1][w]+
                    r.brain[x][y-1][z]*neuronweight[2][w]+
                    r.brain[x][y+1][z]*neuronweight[3][w]+
                    r.brain[x][y][z-1]*neuronweight[4][w]+
                    r.brain[x][y][z+1]*neuronweight[5][w]+

                    r.weight[x][y][z][0]*neuronweight[6][w]+
                    r.weight[x][y][z][1]*neuronweight[7][w]+
                    r.weight[x][y][z][2]*neuronweight[8][w]+
                    r.weight[x][y][z][3]*neuronweight[9][w]+
                    r.weight[x][y][z][4]*neuronweight[10][w]+
                    r.weight[x][y][z][5]*neuronweight[11][w]+

                    r.brain[x][y][z]*neuronweight[12][w]
                    );
                }
            }
        }
    }
}
