#include "2drobots.h"
#define rndp (pos(gen))
#define rndv (vel(gen))
#define rnda (ang(gen))
#define rndw (wei(gen))
using namespace std;

int main(){
    neuronweightinitialize();
    vector<robot> R(numrobots);
    vector<feed> F(numfeeds);
    /*for(int i=0 ; i<1000 ; i++){
        yearlater(R, F);
        printf("step==%d :ageavg%f\n", i,ageavg(R));
    }*/
    weighttrain(R,F);
    return 0;
}
