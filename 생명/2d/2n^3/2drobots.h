#include <bits/stdc++.h>
#define rndp (pos(gen))
#define rndv (vel(gen))
#define rnda (ang(gen))
#define rndw (wei(gen))

using namespace std;

const double inithp = 100000;
const double feedhp = 20;
const double hpdecrease = 1;
const double feedspeed = 0.5;
const double timescale = 0.3;
const double pi = 3.1415926535897932384626;
const double recogdist = 5;
const double fieldsize = 20;
const double feeddist = 1;
const double robotdist = 1;
const double vangle_opposite = 100;
const double tolerance = 1e-8;
const double trainN=5000;

const double WALL = -1;
const double EMPTY = 0;
const double ROBOT = 1;
const double FEED = 2;


const int Bsize = 10; //size of robots' brain = Bsize^3
const int numrobots = 2;
const int numfeeds=0;
const int Squantum = 9;
const int numinput = 13;
const int numoutput = 6;

extern double neuronweight[numinput][numoutput];

extern random_device rd;
extern mt19937 gen;
extern uniform_real_distribution<> pos, vel, ang, wei;
struct feed{
    double x,y,vx,vy;
    double th;
    feed():x(rndp),y(rndp),th(rnda){
        vx = feedspeed * cos(th);
        vy = feedspeed * sin(th);
    }
    void move(){
        x += timescale * vx;
        y += timescale * vy;
        th = rnda;
    }
};

struct robot{
    double x,y,vx,vy;
    double hp;
    double vnorm, th;
    double brain[Bsize+2][Bsize+2][Bsize+2];
    double weight[Bsize+2][Bsize+2][Bsize+2][numoutput];
    double sight[Squantum];
    int age;
    robot():x(rndp),y(rndp),hp(inithp),th(rnda),age(0.),vnorm(1.){ ///vnorm should be updated to rndv!
        vx = vnorm * cos(th);
        vy = vnorm * sin(th);
        for(int i=1;i<=Bsize;i++)
            for(int j=1;j<=Bsize;j++)
                for(int k=1;k<=Bsize;k++)
                    brain[i][j][k] = rndw;

        for(int i=1;i<=Bsize;i++)
            for(int j=1;j<=Bsize;j++)
                for(int k=1;k<=Bsize;k++)
                    for(int l=0;l<numoutput;l++)
                        weight[i][j][k][l] = rndw;
        fill(sight,sight+Squantum,EMPTY);
    }
    robot(const robot &c):x(rndp),y(rndp),hp(inithp),th(rnda),age(0),vnorm(1){
        for(int i=0;i<Bsize+2;i++) for(int j=0;j<Bsize+2;j++) for(int k=0;k<Bsize+2;k++) brain[i][j][k]=c.brain[i][j][k];
        for(int i=0;i<Bsize+2;i++) for(int j=0;j<Bsize+2;j++) for(int k=0;k<Bsize+2;k++) for(int l=0;l<numoutput;l++) weight[i][j][k][l]=c.weight[i][j][k][l];
        fill(sight,sight+Squantum,EMPTY);
    }
    void setth(double t){
        th = t;
        while(th>pi) th-=2.*pi;
        while(th<-pi) th+=2.*pi;
        vx = vnorm * cos(th);
        vy = vnorm * sin(th);
    }
    void move(){
        x += timescale * vx;
        y += timescale * vy;
    }
    double vangle(robot &z){
        double vsize = sqrt(vx*vx+vy*vy);
        if(vsize<tolerance){
            if ( (z.x-x) < 0 ) return vangle_opposite;
            return asin( (z.y-y)  / (sqrt( (z.y-y)*(z.y-y) + (z.x-x)*(z.x-x) ) ) );
        }
        if ( ((z.x-x)*vx + (z.y-y)*vy) < 0 ) return vangle_opposite;
        return asin( ( vx*(z.y-y) - vy*(z.x-x) ) / (vsize * sqrt( (z.y-y)*(z.y-y) + (z.x-x)*(z.x-x) ) ) );
    }
    double vangle(feed &z){
        double vsize = sqrt(vx*vx+vy*vy);
        if(vsize<tolerance){
            if ( (z.x-x) < 0 ) return vangle_opposite;
            return asin( (z.y-y)  / (sqrt( (z.y-y)*(z.y-y) + (z.x-x)*(z.x-x) ) ) );
        }
        if ( ((z.x-x)*vx + (z.y-y)*vy) < 0 ) return vangle_opposite;
        return asin( ( vx*(z.y-y) - vy*(z.x-x) ) / (vsize * sqrt( (z.y-y)*(z.y-y) + (z.x-x)*(z.x-x) ) ) );
    }
};


int collide(vector<robot>& R);
void eat(vector<robot> &R, vector<feed> &F);
int starve(vector<robot> &R);
double dist(robot &u, robot &v);
double dist(robot &u, feed &v);
double dist(feed &u, robot &v);
void sight(vector<robot> &R, vector<feed> &F);
void train(robot &r);
void accel(vector<robot> &R, vector<feed> &F);
void respawn(vector<robot> &R);
void timeflow(vector<robot> &R, vector<feed> &F);
void weighttrain(vector<robot> &R, vector<feed> &F);
void neuronweightinitialize();
void yearlater(vector<robot> &R, vector<feed> &F);
double ageavg(vector<robot> &R);
