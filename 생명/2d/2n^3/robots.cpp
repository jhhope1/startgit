#include "2drobots.h"
using namespace std;
double dist(robot &u, robot &v){
    return sqrt((u.x-v.x)*(u.x-v.x)+(u.y-v.y)*(u.y-v.y));
}
double dist(robot &u, feed &v){
    return sqrt((u.x-v.x)*(u.x-v.x)+(u.y-v.y)*(u.y-v.y));
}
double dist(feed &u, robot &v){
    return sqrt((u.x-v.x)*(u.x-v.x)+(u.y-v.y)*(u.y-v.y));
}

void sight(vector<robot> &R, vector<feed> &F){
    int N=R.size(), M=F.size();
    int now=0;
    double tang, dtemp, cosine, sine;
    double di[Squantum];
    for(int i=0;i<N;i++){
        fill(di, di+Squantum , recogdist+tolerance );
        fill(R[i].sight, R[i].sight+Squantum, EMPTY);
        tang = atan2(R[i].vy,R[i].vx);
        if(R[i].x<recogdist){
            for(int k=0 ; k<Squantum ; k++){
                cosine=cos(tang)*cos(pi/2-pi/Squantum*k)-sin(tang)*sin(pi/2-pi/Squantum*k);
                if(cosine*recogdist+R[i].x<0){
                    di[k]=-R[i].x/cosine;
                    R[i].sight[k]=WALL;
                }
            }
        }
        if(R[i].x>fieldsize-recogdist){
            for(int k=0 ; k<Squantum ; k++){
                cosine=cos(tang)*cos(pi/2-pi/Squantum*k)-sin(tang)*sin(pi/2-pi/Squantum*k);
                if(cosine*recogdist+R[i].x>fieldsize){
                    di[k]=(fieldsize-R[i].x)/cosine;
                    R[i].sight[k]=WALL;
                }
            }
        }
        if(R[i].y<recogdist){
            for(int k=0 ; k<Squantum ; k++){
                sine=sin(tang)*cos(pi/2-pi/Squantum*k)+cos(tang)*sin(pi/2-pi/Squantum*k);
                if(sine*recogdist+R[i].x<0){
                    di[k]=-R[i].x/sine;
                    R[i].sight[k]=WALL;
                }
            }
        }
        if(R[i].y>fieldsize-recogdist){
            for(int k=0 ; k<Squantum ; k++){
                sine=sin(tang)*cos(pi/2-pi/Squantum*k)+cos(tang)*sin(pi/2-pi/Squantum*k);
                if(sine*recogdist+R[i].x>fieldsize){
                    di[k]=(fieldsize-R[i].x)/sine;
                    R[i].sight[k]=WALL;
                }
            }
        }

        for(int j=0;j<N;j++){
            dtemp = dist(R[i],R[j]);
            if( dtemp > recogdist ) continue;
            if( i==j ) continue;
            tang = R[i].vangle(R[j]);
            if( tang > vangle_opposite-1 ) continue;
            now = min( Squantum-1 , (int)(( tang + pi/2.) / (pi/Squantum) ));
            if(di[now] > dtemp){
                di[now] = dtemp;
                R[i].sight[now] = ROBOT;
            }
        }

        for(int j=0;j<M;j++){
            dtemp = dist(R[i],F[j]);
            if( dtemp > recogdist ) continue;
            tang = R[i].vangle(F[j]);
            if( tang > vangle_opposite-1 ) continue;
            now = min( Squantum-1 , (int)(( tang + pi/2.) / (pi/Squantum) ));
            if(di[now] > dtemp){
                di[now] = dtemp;
                R[i].sight[now] = FEED;
            }
        }
    }
    for(robot &r : R){
        int i=0;
        int sq=int(sqrt(Squantum-1))+1;
        for(int y=0 ; y<sq ; y++){
            for(int z=0 ; z<sq ; z++){
                r.brain[0][(Bsize-sq)/2+y][(Bsize-sq)/2+z]=r.sight[i];/////0으로 해도 되는거갓트다.
                if(i==Squantum)break;
                i++;
            }
        }
    }
}
