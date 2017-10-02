#include <bits/stdc++.h>
using namespace std;
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> dist(0,1);
const double initlive=20;
const double feedspeed=0.1;
const double timescale=5;// 시간이 지날때마다 시간 스케일을 어케 할거냐?
const double pi=3.141592;
const int N=1;
const double recogdistance=10;
const double feeddistance=1;
const int feedN=1;
const double feedvalue=1;
const double wall=30;//경기장의 크기이다.
double out[6]={0};
const int sightquantum=9; //시야를 얼마나 쪼개서 볼 것이냐(즉 180이면 1도마다 볼 수 있다. 로봇 시야에서 +-90도로 볼수있다고 하자)
int die=0;//로봇들이 죽은 횟수
double robotposition[N][3]= {0}; //n th robot's position(angle, x, y)
double robotspeed[N][2]= {0}; //n th robot's speed
double robotsight[N][sightquantum]= {0}; //n th robot's sight(robot: 1, wall: 2, nothing: 0, feed: -1)
double feedposition[feedN][2]= {0}; //마찬가지로 먹이의 position이다.
double live[N]= {0}; // live가 0보다 작아지면 뒤진다.죽은 직후에는 포지션이 (-100, -100)이 되도록 한다.
void livefunc();
void accelation(int n, double d_angle, double af, double al);//앞방향 가속도가 주어지고, left 방향 가속도가 주어질때 다음 위치와의 차이를 반환한다.
void robotsightfunc(int n);
void timeflow();
const int width=5;
double brain[N][width][width][width]= {0}; //각 개체들의 뇌이다. 즉 width^3 크기의 뇌와 같다. 특히 뉴런들은 x,y,z차이가 1 이하인 것들과만 소통 가능하다..
double weight[N][width][width][width][6]= {0}; //모두 6가지 방향에서 뉴런들의 output이 들어온다. 또한 자기자신의 값도 있다.(이것을 weight 크기를 7로 늘려서 해결할지 아니면 다른 방식으로 해결할지를 고민이다. 망각이 필요하다. 따라서 weight의 크기는 brain의 6배이다. 가벼워서 훈련도 빠를 것 같다.
double neuronweight[6][13]= {0}; //뉴런의 weight를 어떻게 바꿀지 생각하는 곳이다. 13개(6개의 initial weight, 6개의 neuron값, 그리고 자기자신의 neuron값)의 인풋과 6개의 output(weight 변화량)dl dlTek.
double tangentstep(double x);
int outofrange(int x);
/*      1
    0   brain   3       2는 brain 아래(즉 뚫고들어가는 방향), 6은 brain 위(즉 뚫고 나오는 방향)이다.
        4
*/

//File*robotinput = fopen("아무말대잔치", "r");
void think();//think를 여러번할 수도 있고 아닐수도 있다. 인간은 사고할때 긴박한 상황에서 는 짧은 시간안에 대처하고 아니면 더 긴 시간 사고한다.
void weightchange();// neuron weight로 모든 부분의 weight를 바꿔주는 곳
double* neuronmatmul(double *a);//6*13과 13을 matmul
double myfunction(double x, double y);
double gogogo();//2d현장으로 투입되었다. 남은 인원을 보고해라!!!
void neuronweighttrain();
void braintrain();//brain초기화를 잘해준다. 현재 뇌는 초기화가 무진장 잘되잇다. 그래서 어느정도는 초기화가 잘 되어있어야 훈련을 해도 발산하거나 이상하기 되지 않는다. 사실 나도 잘 모른다. 그냥 이게 되면 되는거고 안되면 안되는거다.
void positioninitialize();
void tfeedposition();
void callweight();
double tangentstep(double x);
void weightinitialize();
void neuronweightinitialize();
double angleinitial(double x)
{
    while(1)
    {
        if(x>(2*pi+0.01))
            x-=2*pi;
        else if(x<0)
            x+=2*pi;
        else
            return x;
    }
}
void accelation(int n, double d_angle, double af, double al)
{
    robotposition[n][0]+=d_angle*timescale;
    robotposition[n][0]=angleinitial(robotposition[n][0])*timescale;
    robotspeed[n][0]+=af*cos(robotposition[n][0])-al*sin(robotposition[n][0]);
    robotspeed[n][1]+=af*sin(robotposition[n][0])+al*cos(robotposition[n][0]);
    robotposition[n][2]+=robotspeed[n][1]*timescale;
}
void livefunc()//여기서 distance는 거리의 제곱을 의미한다. 주의 요망.
{
    for(int i=0 ; i<N ; i++)
    {
        if(live[i]<0)
        {
            continue;
        }
        live[i]-=1;
        for(int j=0 ; j<feedN ; j++)
        {
            double dsquare=(robotposition[i][1]-feedposition[j][0])*(robotposition[i][1]-feedposition[j][0])+(robotposition[i][2]-feedposition[j][1])*(robotposition[i][2]-feedposition[j][1]);
            if(dsquare<feeddistance*feeddistance)
            {
                live[i]+=feedvalue;
            }
        }
        for(int j=i+1 ; j<N ; j++)
        {
            double dsquare=(robotposition[i][1]-robotposition[j][1])*(robotposition[i][1]-robotposition[j][1])+(robotposition[i][2]-robotposition[j][2])*(robotposition[i][2]-robotposition[j][2]);
            if(dsquare<feeddistance*feeddistance)
            {
                live[i]=-1;
                live[j]=-1;
            }
        }
        if(robotposition[i][0]<0 || robotposition[i][0]>wall ||robotposition[i][1]<0 || robotposition[i][1]>wall )
            live[i]=-1;
    }
}
void robotsightfunc(int n)
{
    double distance[sightquantum]= {0}; //-90도부터 +90도 까지 쪼개진 것이다.
    for(int i=0 ; i<sightquantum ; i++)
    {
        distance[i]=recogdistance*recogdistance+1;
    }
    for(int i=0 ; i<N ; i++)
    {
        double distancetemp=(robotposition[i][1]-robotposition[n][1])*(robotposition[i][1]-robotposition[n][1])+(robotposition[i][2]-robotposition[n][2])*(robotposition[i][2]-robotposition[n][2]);
        if(distancetemp<recogdistance*recogdistance)
        {
            double tangent=(robotposition[i][2]-robotposition[n][2])/(robotposition[i][1]-robotposition[n][1]);
            double relativetangent=(tangent-tan(robotposition[n][0]))/(1+tangent*tan(robotposition[n][0]));
            if((robotposition[i][1]-robotposition[n][1])*cos(robotposition[n][0])+(robotposition[i][2]-robotposition[n][2])*sin(robotposition[n][0])>0)
            {
                for(int j=0 ; j<sightquantum ; j++)
                {
                    if(tan(-pi/2+j*pi/double(sightquantum))<relativetangent && tan(-pi/2+(j+1)*pi/double(sightquantum))>relativetangent)
                    {
                        if(distance[j]>distancetemp)
                        {
                            distance[j]=distancetemp;
                            robotsight[n][j]=1;
                        }
                    }
                }
            }
        }
    }
    for(int i=0 ; i<feedN ; i++)
    {
        double distancetemp=(robotposition[n][1]-feedposition[i][0])*(robotposition[n][1]-feedposition[i][0])+(robotposition[n][2]-robotposition[i][1])*(robotposition[n][2]-robotposition[i][1]);
        if(distancetemp<recogdistance*recogdistance)
        {
            double tangent=(robotposition[n][2]-feedposition[i][1])/(robotposition[n][1]-robotposition[i][0]);
            double relativetangent=(tangent-tan(robotposition[n][0]))/(1+tangent*tan(robotposition[n][0]));
            if((-robotposition[n][1]+feedposition[i][0])*cos(robotposition[n][0])+(-robotposition[n][2]+feedposition[i][1])*sin(robotposition[n][0])>0)
            {
            for(int j=0 ; j<sightquantum ; j++)
                {
                    if(tan(-pi/2+j*pi/sightquantum)<relativetangent && tan(-pi/2+(j+1)*pi/sightquantum)>relativetangent)
                    {
                        if(distance[j]>distancetemp)
                        {
                            distance[j]=distancetemp;
                            robotsight[n][j]=-1;
                        }
                    }
                }
            }
        }
    }
    if(robotposition[n][1]<recogdistance)
    {
        for(int i=0 ; i<sightquantum ; i++)
        {
            double cosine=cos(robotposition[n][0])*cos(pi/2-pi/sightquantum*i)-sin(robotposition[n][0])*sin(pi/2-pi/sightquantum*i);
            if(cosine*recogdistance+robotposition[n][1]<0)
            {
                distance[i]=-robotposition[n][1]/cosine;
                robotsight[n][i]=2;
            }
        }
    }
    if(robotposition[n][1]>wall-recogdistance)
    {
        for(int i=0 ; i<sightquantum ; i++)
        {
            double cosine=cos(robotposition[n][0])*cos(pi/2-pi/sightquantum*i)-sin(robotposition[n][0])*sin(pi/2-pi/sightquantum*i);
            if(cosine*recogdistance+robotposition[n][1]>wall)
            {
                distance[i]=(wall-robotposition[n][1])/cosine;
                robotsight[n][i]=2;
            }
        }
    }
    if(robotposition[n][2]<recogdistance)
    {
        for(int i=0 ; i<sightquantum ; i++)
        {
            double sine=sin(robotposition[n][0])*cos(pi/2-pi/sightquantum*i)+cos(robotposition[n][0])*sin(pi/2-pi/sightquantum*i);
            if(sine*recogdistance+robotposition[n][1]<0)
            {
                distance[i]=-robotposition[n][1]/sine;
                robotsight[n][i]=2;
            }
        }
    }
    if(robotposition[n][2]>wall-recogdistance)
    {
        for(int i=0 ; i<sightquantum ; i++)
        {
            double sine=sin(robotposition[n][0])*cos(pi/2-pi/sightquantum*i)+cos(robotposition[n][0])*sin(pi/2-pi/sightquantum*i);
            if(sine*recogdistance+robotposition[n][1]>wall)
            {
                distance[i]=(wall-robotposition[n][1])/sine;
                robotsight[n][i]=2;
            }
        }
    }
}


double tangentstep(double x)
{
    if(x>3)
        return 3;
    if(x<-3)
        return -3;
    return x;
}
int outofrange(int x)
{
    if(x==0||x==width)
    {
        return 0;
    }
    return 1;
}
void think()
{
    double temp=0;
    for(int asl=0 ; asl<N ; asl++)
    {
        robotsightfunc(asl);
        int rootq=int(sqrt(sightquantum));
        int yay=0;

        while(1){//뇌에 시각 정보 입력
            for(int j=0 ; j<rootq ; j++){
                brain[asl][width/2-rootq/2+j][yay%rootq][0]=robotsight[asl][yay];

            }
            yay++;
            if(yay==sightquantum)
                break;
        }
        for(int i=1 ; i<width-1 ; i++)
        {
            for(int j=1 ; j<width-1 ; j++)
            {
                for(int k=1 ; k<width-1 ; k++)
                {
                    temp=0;
                    temp=+weight[asl][i][j][k][0]*brain[asl][i-1][j][k]+weight[asl][i][j][k][1]*brain[asl][i][j-1][k]+weight[asl][i][j][k][2]*brain[asl][i][j][k-1]+weight[asl][i][j][k][3]*brain[asl][i+1][j][k]+weight[asl][i][j][k][4]*brain[asl][i][j+1][k]+weight[asl][i][j][k][5]*brain[asl][i][j][k+1];
                    brain[asl][i][j][k]=myfunction(brain[asl][i][j][k], temp);
                }
            }
        }
        for(int i=1 ; i<width-1; i++)
        {
            for(int j=1 ; j<width-1 ; j++)
            {
                temp=0;
                temp=+weight[asl][i][j][0][0]*brain[asl][i-1][j][0]+weight[asl][i][j][0][1]*brain[asl][i][j-1][0]+weight[asl][i][j][0][3]*brain[asl][i+1][j][0]+weight[asl][i][j][0][4]*brain[asl][i][j+1][0]+weight[asl][i][j][0][5]*brain[asl][i][j][1];
                brain[asl][i][j][0]=myfunction(brain[asl][i][j][0], temp);
                temp=0;
                temp=+weight[asl][i][j][width-1][0]*brain[asl][i-1][j][width-1]+weight[asl][i][j][width-1][1]*brain[asl][i][j-1][width-1]+weight[asl][i][j][width-1][2]*brain[asl][i][j][width-2]+weight[asl][i][j][width-1][3]*brain[asl][i+1][j][width-1]+weight[asl][i][j][width-1][4]*brain[asl][i][j+1][width-1];
                brain[asl][i][j][width-1]=myfunction(brain[asl][i][j][width-1], temp);
                temp=0;
                temp=+weight[asl][0][i][j][1]*brain[asl][0][i-1][j]+weight[asl][0][i][j][2]*brain[asl][0][i][j-1]+weight[asl][0][i][j][3]*brain[asl][1][i][j]+weight[asl][0][i][j][4]*brain[asl][0][i+1][j]+weight[asl][0][i][j][5]*brain[asl][0][i][j+1];
                brain[asl][0][i][j]=myfunction(brain[asl][0][i][j], temp);
                temp=0;
                temp=+weight[asl][width-1][i][j][0]*brain[asl][width-2][i][j]+weight[asl][width-1][i][j][1]*brain[asl][width-1][i-1][j]+weight[asl][width-1][i][j][2]*brain[asl][width-1][i][j-1]+weight[asl][width-1][i][j][4]*brain[asl][width-1][i+1][j]+weight[asl][width-1][i][j][5]*brain[asl][width-1][i][j+1];
                brain[asl][width-1][i][j]=myfunction(brain[asl][width-1][i][j], temp);
                temp=0;
                temp=+weight[asl][i][0][j][0]*brain[asl][i-1][0][j]+weight[asl][i][0][j][2]*brain[asl][i][0][j-1]+weight[asl][i][0][j][3]*brain[asl][i+1][0][j]+weight[asl][i][0][j][4]*brain[asl][i][1][j]+weight[asl][i][0][j][5]*brain[asl][i][0][j+1];
                brain[asl][i][0][j]=myfunction(brain[asl][i][0][j], temp);
                temp=0;
                temp=+weight[asl][i][width-1][0][0]*brain[asl][i-1][width-1][j]+weight[asl][i][width-1][j][1]*brain[asl][i][width-2][j]+weight[asl][i][width-1][j][2]*brain[asl][i][width-1][j-1]+weight[asl][i][width-1][j][3]*brain[asl][i+1][width-1][j]+weight[asl][i][width-1][j][5]*brain[asl][i][width-1][j+1];
                brain[asl][i][j][width-1]=myfunction(brain[asl][i][j][width-1], temp);
                //너무 귀찮아서 그냥 여기까지만 함. 어차피 이것도 안해도 굴러는 갔을듯.
            }
        }
    }
}
void timeflow()
{
    think();
    for(int i=0 ; i<N ; i++)
    {
        if(live[i]<0)
        {
            die++;
            robotposition[i][0]=2.0*pi*dist(gen);
            robotposition[i][1]=wall*dist(gen);
            robotposition[i][2]=wall*dist(gen);
            robotspeed[i][0]=dist(gen);
            robotspeed[i][1]=dist(gen);
            think();
            live[i]=initlive;
        }
        accelation(i, brain[i][width/2][0][width/2],brain[i][0][width/2][width/2], brain[i][width-1][width/2][width/2]);//////////////////////////
    }

    /*for (int j=0 ; j<width ; j++){
        for (int k=0 ; k<width ; k++){
            printf("%f\t", brain[0][j][k][0]);
        }
            printf("\n");
    }
    printf("\n줄바꿈");*/

    //printf("\n\n");
    weightchange();
    tfeedposition();
    livefunc();
}
void weightchange()
{
    for(int asl=0 ; asl<N ; asl++)
    {
        for(int i=0 ; i<width ; i++)
        {
            for(int j=0 ; j<width ; j++)
            {
                for(int k=0 ; k<width ; k++)
                {
                    double temp[13]= {0};
                    for(int l=0 ; l<6 ; l++){
                        temp[l]=weight[asl][i][j][k][l];}
                    temp[6]=brain[asl][i][j][k];
                    if(outofrange(i-1))
                        temp[7]=brain[asl][i-1][j][k];
                    if(outofrange(i+1))
                        temp[8]=brain[asl][i+1][j][k];
                    if(outofrange(j-1))
                        temp[9]=brain[asl][i][j-1][k];
                    if(outofrange(j+1))
                        temp[10]=brain[asl][i][j+1][k];
                    if(outofrange(k-1))
                        temp[11]=brain[asl][i][j][k-1];
                    if(outofrange(k+1))
                        temp[12]=brain[asl][i][j][k+1];
                    for(int l=0 ; l<6 ; l++)
                    {
                        weight[asl][i][j][k][l]=tangentstep(weight[asl][i][j][k][l]+*(neuronmatmul(temp)+l));
                    }
                }
            }
        }
    }
}
double* neuronmatmul(double* a)
{
    for (int i=0 ; i<6 ; i++)
    {
        double temp=0;
        for( int j=0 ; j<13 ; j++)
            temp+=neuronweight[i][j]*(*(a+j));
        out[i]=tanh(temp/100.);
    }
    return out;
}
double myfunction(double x, double y)
{
    if((y)<1&&(y)>-1)
        return x;
    return tangentstep(y/10+x);
}
void positioninitialize(){
    for(int i=0 ; i<N ; i++){//robot initialize
        robotposition[i][0]=2.0*pi*dist(gen);
        robotposition[i][1]=wall*dist(gen);
        robotposition[i][2]=wall*dist(gen);
    }
    for(int i=0 ; i<N ; i++){//robot initialize
        robotspeed[i][0]=dist(gen);
        robotspeed[i][1]=dist(gen);
    }
    for(int i=0 ; i<feedN ; i++){//robot initialize
        feedposition[i][0]=wall*dist(gen);
        feedposition[i][1]=wall*dist(gen);
    }
    for (int i=0 ; i<N ; i++)
        live[i]=initlive;

}
void weightinitialize(){
    for(int i=0 ; i<N ; i++){
        for(int j=0 ; j<width ; j++){
            for(int k=0 ; k<width ; k++){
                for(int l=0 ; l<width ; l++){
                    for(int n=0 ; n<6 ; n++){
                        weight[i][j][k][l][n]=0.01*(dist(gen)-0.5);
                    }
                }
            }
        }
    }
    for(int i=0 ; i<N ; i++){
        for(int j=0 ; j<width ; j++){
            for(int k=0 ; k<width ; k++){
                for(int l=0 ; l<width ; l++){
                    brain[i][j][k][l]=0.01*(dist(gen)-0.5);
                }
            }
        }
    }
    //추후에 뒤로 따로 빼서 작성할것//yay 지금이 추후다 시발 밑에 있음
}

void neuronweightinitialize(){
    for(int i=0 ; i<6 ; i++){
        for(int j=0 ; j<13 ; j++){
            neuronweight[i][j]=0.01*(dist(gen)-0.5);
        }
    }
}
void tfeedposition(){
    for(int i=0 ; i<feedN ; i++){
        double temp=feedspeed*dist(gen);
        feedposition[i][0]+=temp;
        if(feedposition[i][0]<0 || feedposition[i][0]>wall)
            feedposition[i][0]-=2*temp;
        temp=feedspeed*dist(gen);
        feedposition[i][1]+=temp;
        if(feedposition[i][1]<0 || feedposition[i][1]>wall)
            feedposition[i][1]-=2*temp;
    }
}
double gogogo(){
    double j=0;
    for(int i=0 ; i<N ; i++){
        if (live[i]<0)
            continue;
        j++;
    }
    return j;
}

void neuronweighttrain()
{
    for(int i=0 ; i<6 ; i++){
        for(int j=0 ; j<13 ; j++){
            double temp1=0;
            double localdie[3]={0};
            die=0;
            neuronweight[i][j]-=0.1;
            weightinitialize();
            positioninitialize();
            for(int step=0 ; step<1000 ; step++){
                timeflow();
                if(step==500)temp1=die;
            }
            localdie[0]=die-temp1;
            die=0;
            neuronweight[i][j]+=0.1;
            weightinitialize();
            positioninitialize();
            for(int step=0 ; step<1000 ; step++){
                if(step==500)temp1=die;
                timeflow();
            }
            localdie[1]=die-temp1;
            die=0;
            neuronweight[i][j]+=0.1;
            weightinitialize();\
            positioninitialize();
            for(int step=0 ; step<1000 ; step++){
                if(step==500)temp1=die;
                timeflow();
            }
            localdie[2]=die-temp1;
            //if(localdie[0]+localdie[2]-2*localdie[0]>0){
            //    neuronweight[i][j]=tangentstep(neuronweight[i][j]-0.1-(localdie[2]-localdie[0])/(localdie[0]+localdie[2]-2*localdie[1])/20.0);
           // }
            //else{
            double mim=0;
                if(localdie[0]<localdie[2])
                    mim=-2;
                if(localdie[1]<localdie[2])
                    mim=-1;
                neuronweight[i][j]=tangentstep(neuronweight[i][j]+mim*0.1);
            //}
            printf("(%d,%d)", i,j);
            printf("%f, %f, %f\n", localdie[0], localdie[1], localdie[2]);
            for(int k=0 ; k<6 ; k++){
                for(int l=0 ; l<13 ; l++){
                    printf("%.3f\t", neuronweight[k][l]);
                }
                printf("\n");
            }
        }
    }
}

int main()
{
    weightinitialize();
    neuronweightinitialize();
    positioninitialize();
    die=0;
    for(int i=0 ; i<100 ; i++){
        neuronweighttrain();
    }
}
