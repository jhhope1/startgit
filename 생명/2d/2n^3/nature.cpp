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
    for(int i=0 ; i<R.size() ; i++){
        R[i].setth(R[i].th + R[i].brain[Bsize/2][Bsize/2][Bsize/2]);
        R[i].move();
        //printf("Robot %d: (x,y)=(%f,%f), velocity=%f, age=%d\n",i, R[i].x, R[i].y, sqrt(R[i].vx*R[i].vx+R[i].vy*R[i].vy), R[i].age);
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
    /*printf("R[0]'s brain\n");
    for(int i=0 ; i<Bsize+2 ; i++){
        for(int j=0 ; j<Bsize+2 ; j++){
            for(int k=0 ; k<Bsize+2 ; k++){
                printf("%.2f\t", R[0].brain[i][j][k]);
            }
            printf("\n");
        }
        printf("%d¹øÂ° Ãþ\n",i);
    }
    printf("R[0]'s sight\n");
    for(int i=0 ; i<Squantum ; i++)printf("%.2f ", R[0].sight[i]);
    printf("\n");
    printf("R[0]'s location=(%.2f,%.2f), angle=%f degree\n", R[0].x, R[0].y, R[0].th*180/pi);
    */
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
