#include <bits/stdc++.h>
using namespace std;
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> dist(0,1);
const double initlive=20;
const double feedspeed=0.1;
const double timescale=5;// �ð��� ���������� �ð� �������� ���� �Ұų�?
const double pi=3.141592;
const int N=1;
const double recogdistance=10;
const double feeddistance=1;
const int feedN=1;
const double feedvalue=1;
const double wall=30;//������� ũ���̴�.
double out[6]={0};
const int sightquantum=9; //�þ߸� �󸶳� �ɰ��� �� ���̳�(�� 180�̸� 1������ �� �� �ִ�. �κ� �þ߿��� +-90���� �����ִٰ� ����)
int die=0;//�κ����� ���� Ƚ��
double robotposition[N][3]= {0}; //n th robot's position(angle, x, y)
double robotspeed[N][2]= {0}; //n th robot's speed
double robotsight[N][sightquantum]= {0}; //n th robot's sight(robot: 1, wall: 2, nothing: 0, feed: -1)
double feedposition[feedN][2]= {0}; //���������� ������ position�̴�.
double live[N]= {0}; // live�� 0���� �۾����� ������.���� ���Ŀ��� �������� (-100, -100)�� �ǵ��� �Ѵ�.
void livefunc();
void accelation(int n, double d_angle, double af, double al);//�չ��� ���ӵ��� �־�����, left ���� ���ӵ��� �־����� ���� ��ġ���� ���̸� ��ȯ�Ѵ�.
void robotsightfunc(int n);
void timeflow();
const int width=5;
double brain[N][width][width][width]= {0}; //�� ��ü���� ���̴�. �� width^3 ũ���� ���� ����. Ư�� �������� x,y,z���̰� 1 ������ �͵���� ���� �����ϴ�..
double weight[N][width][width][width][6]= {0}; //��� 6���� ���⿡�� �������� output�� ���´�. ���� �ڱ��ڽ��� ���� �ִ�.(�̰��� weight ũ�⸦ 7�� �÷��� �ذ����� �ƴϸ� �ٸ� ������� �ذ������� ����̴�. ������ �ʿ��ϴ�. ���� weight�� ũ��� brain�� 6���̴�. �������� �Ʒõ� ���� �� ����.
double neuronweight[6][13]= {0}; //������ weight�� ��� �ٲ��� �����ϴ� ���̴�. 13��(6���� initial weight, 6���� neuron��, �׸��� �ڱ��ڽ��� neuron��)�� ��ǲ�� 6���� output(weight ��ȭ��)dl dlTek.
double tangentstep(double x);
int outofrange(int x);
/*      1
    0   brain   3       2�� brain �Ʒ�(�� �հ���� ����), 6�� brain ��(�� �հ� ������ ����)�̴�.
        4
*/

//File*robotinput = fopen("�ƹ�������ġ", "r");
void think();//think�� �������� ���� �ְ� �ƴҼ��� �ִ�. �ΰ��� ����Ҷ� ����� ��Ȳ���� �� ª�� �ð��ȿ� ��ó�ϰ� �ƴϸ� �� �� �ð� ����Ѵ�.
void weightchange();// neuron weight�� ��� �κ��� weight�� �ٲ��ִ� ��
double* neuronmatmul(double *a);//6*13�� 13�� matmul
double myfunction(double x, double y);
double gogogo();//2d�������� ���ԵǾ���. ���� �ο��� �����ض�!!!
void neuronweighttrain();
void braintrain();//brain�ʱ�ȭ�� �����ش�. ���� ���� �ʱ�ȭ�� ������ �ߵ��մ�. �׷��� ��������� �ʱ�ȭ�� �� �Ǿ��־�� �Ʒ��� �ص� �߻��ϰų� �̻��ϱ� ���� �ʴ´�. ��� ���� �� �𸥴�. �׳� �̰� �Ǹ� �Ǵ°Ű� �ȵǸ� �ȵǴ°Ŵ�.
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
void livefunc()//���⼭ distance�� �Ÿ��� ������ �ǹ��Ѵ�. ���� ���.
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
    double distance[sightquantum]= {0}; //-90������ +90�� ���� �ɰ��� ���̴�.
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

        while(1){//���� �ð� ���� �Է�
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
                //�ʹ� �����Ƽ� �׳� ��������� ��. ������ �̰͵� ���ص� ������ ������.
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
    printf("\n�ٹٲ�");*/

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
    //���Ŀ� �ڷ� ���� ���� �ۼ��Ұ�//yay ������ ���Ĵ� �ù� �ؿ� ����
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
