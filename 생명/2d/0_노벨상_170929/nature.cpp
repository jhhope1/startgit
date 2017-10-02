#include "2drobots.h"

int collide(vector<robot> &R){
    int N=R.size();
    vector<bool> B(N,false);
    for(int i=N;i--;){
        for(int j=i;j--;){
            B[i]=B[j]=(dist(R[i],R[j])<robotdist);
        }
    }
    for(int i=N;i--;){
        if(R[i].x < 0 || R[i].x > fieldsize || R[i].y < 0 || R[i].y > fieldsize) B[i]=true;
    }
    for(int i=N;i--;){
        if(B[i]) R.erase(R.begin()+i);
    }
    return N-R.size();
}

void eat(vector<robot> &R, vector<feed> &F){
    int N=R.size(), M=F.size();
    for(int i=0;i<N;i++){
        for(int j=0;j<M;j++){
            if(dist(R[i],F[j])<feeddist){
                R[i].hp+=feedhp;
            }
        }
    }
}

int starve(vector<robot> &R){
    int N=R.size();
    for(int i=N;i--;){
        R[i].hp-=hpdecrease;
        if(!(R[i].hp>0)) R.erase(R.begin()+i);
    }
    return N-R.size();
}
void accel(vector<robot> &R, vector<feed> &F){
    for(robot &r : R){
        r.setth(r.th + r.brain[Bsize/2][Bsize/2][Bsize/2]);
        r.move();
    }
    for(feed &f : F){
        f.move();
    }
}
void respawn(vector<robot> &R, int goal){
    int D = goal - R.size();
    for(;D--;){
        R.push_back(robot(*max_element(R.begin(),R.end(),[](robot &u, robot &v)->bool{
                                        return u.age<v.age;
                                       })));
    }
}
void timeflow(vector<robot> &R, vector<feed> &F){
    int N=R.size();
    collide(R);
    eat(R,F);
    starve(R);
    sight(R,F);
    for(robot &r : R) train(r);
    accel(R,F);
    for(robot &r : R) r.age+=1;
    respawn(R,N);
}
