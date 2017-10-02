#include "2drobots.h"
using namespace std;
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> pos(0,fieldsize), vel(0,1), ang(0,pi*2), wei(-1,1);
double neuronweight[numinput][numoutput];
