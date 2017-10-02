#include "2drobots.h"
void neuronweightinitialize();
void getdata(vector<robot> &R);//훈련된 데이터 받기
void memodata(vector<robot> &R);
void neuronweighttrain(vector<robot> &R);
void weighttrain(vector<robot> &R,vector<feed> &F);
void yearlater(vector<robot> &R,vector<feed> &F);
double step(double x);
double ageavg(vector<robot> &R);

void neuronweightinitialize(){
    for(int i=0 ; i<numinput ; i++){
        for(int j=0 ; j<numoutput ; j++){
            neuronweight[i][j]=rndw;
        }
    }
}

void getdata(vector<robot> &R){
    for(robot &r : R){
        FILE* weightdata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\weight.txt", "rt");
        FILE* neuronweightdata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\neuronweight.txt","rt");
        FILE* braindata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\brain.txt","rt");
        for(int j=0 ; j<Bsize*Bsize*Bsize*numoutput ; j++){
			fscanf(weightdata, "%.6lf", (*(***r.weight+j)));
        }
        for(int j=0 ; j<numinput*numoutput ; j++){
			fscanf(neuronweightdata, "%.6lf", (*(*neuronweight+j)));
        }
        for(int j=0 ; j<Bsize*Bsize*Bsize ; j++){
			fscanf(braindata, "%.6lf", (*(**r.brain+j)));
        }
        fclose(weightdata);
        fclose(neuronweightdata);
        fclose(braindata);
    }
}

void memodata(vector<robot> &R){
    FILE* weightdata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\weight.txt", "wt");
    FILE* neuronweightdata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\neuronweight.txt","wt");
    FILE* braindata=fopen("C:\\Users\\김재환\\Desktop\\생명\\2d\\savevalue\\brain.txt","wt");
    robot r=robot(*max_element(R.begin(),R.end(),[](robot &u, robot &v)->bool{
                                        return u.age<v.age;
                                       }));
    for(int j=0 ; j<Bsize*Bsize*Bsize*numoutput ; j++){
        fprintf(weightdata, "%f", float(*(***r.weight+j)));
    }
    for(int j=0 ; j<numinput*numoutput ; j++){
        fprintf(neuronweightdata, "%f", float(*(*neuronweight+j)));
    }
    for(int j=0 ; j<Bsize*Bsize*Bsize ; j++){
        fprintf(braindata, "%f", float(*(**r.brain+j)));
    }
    fclose(weightdata);
    fclose(neuronweightdata);
    fclose(braindata);
}
void yearlater(vector<robot> &R, vector<feed> &F){
    for(int step=0 ; step<trainN ; step++){
        //printf("step=%d\n\n", step );
        timeflow(R,F);
    }
}
double ageavg(vector<robot> &R){
    int age=0;
    for(robot &r : R){
        age+=r.age;
    }
    return double(age)/double(numrobots);
}
double step(double x){
    if(x>1)return 1;
    else if(x<-1)return -1;
    else return x;
}
void weighttrain(vector<robot> &R,vector<feed> &F){
    getdata(R);
    double age[3]={0};
    for(int i=0 ; i<numinput ; i++){
        for(int j=0 ; j<numoutput ; j++){
            yearlater(R,F);
            age[1]=ageavg(R);
            std::cout << ageavg(R)<<std::endl;
            neuronweight[i][j]+=0.1;
            yearlater(R,F);
            age[2]=ageavg(R);
            neuronweight[i][j]-=0.2;
            yearlater(R,F);
            age[0]=ageavg(R);
            neuronweight[i][j]+=0.1;
            if(age[1]>age[2]&&age[1]>age[0])continue;
            else if(age[2]>age[1]&&age[2]>age[0]){
                neuronweight[i][j]+=0.1;
                continue;
            }
            neuronweight[i][j]-=0.1;
            memodata(R);
        }
    }
}
