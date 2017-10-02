#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<time.h>
#define hidn 1 //은닉층의 개수: 따라서 층수의 총개수는 hidn+2이다.
#define rate 0.1//말그대로 rate: gradient에 얼마를 곱할 것인가?
double**layer = (double**)malloc(sizeof(double*) * (hidn+2));//단 layer[0]는 입력층 즉 get을 의미, layer[hidn+1]은 출력층을 의미한다.
int layn[hidn + 2] = {784, 10, 10};//차례로, 입력층뉴런수, hid층의 뉴런수, 출력층의 뉴런수
double***weight = (double***)malloc(sizeof(double**) * (hidn + 1));//weight[i][j][k]는 layer[i]에서 layer[i+1]로 가는 가중치로, layer[i][j]와 layer[i][k]를 이어주는 가중치이다.
double***gradient = (double***)malloc(sizeof(double**) * (hidn + 1));//round(E)/round(weight[i][j][k])이다.
double *real=(double*)malloc(sizeof(int)*(layn[hidn+1]));
double error;
double bestcorrect;
unsigned char data[784];
unsigned char num;
double ***bestweight = (double***)malloc(sizeof(double**) * (hidn + 1));
FILE *image;
FILE *label;
FILE *image2;
FILE *label2;
FILE *out = fopen("output.txt", "wb");
void laymake();//layer들을 만들어주는 것.
void weight_and_gradient_make();//weight를 만든다.
double sigmoid(double);//활성함수는 일단 sigmoid함수로 한다.
void make_next_layer(int i);//layer[i]를 weight[i]들을 곱해 layer[i+1]을 만드는 과정
void make_all_layer();//모든 층을 만든다.
void weight_initialization();//가중치를 무작위로 초기화시켜준다.
void backpropagation(int);//역전파
double geterror();//error을 얻는 함수
void getweight();
void memoweight();
void memobestweight();
void printcorrect();
void get_trainset(int);
void setting();

void setting()
{
	fopen_s(&image,"train-images.idx3-ubyte", "rb");
	fopen_s(&label,"train-labels.idx1-ubyte", "rb");
	fopen_s(&image2,"t10k-images.idx3-ubyte", "rb");
	fopen_s(&label2,"t10k-labels.idx1-ubyte", "rb");
	if (image == 0 || label == 0 || image2 == 0 || label2 == 0)
	{
		printf("File opening error12");
	}
}

void train(int stepcnt)
{
	int i;
	int backpropagatecnt = 0;
	double tmpcost;
	while (true) {
		backpropagatecnt++;
		printf("역전파 %d번째 수행중...\n", backpropagatecnt);
		for (i = 0; i<60000; i++) backpropagation(i);
		tmpcost = geterror();
		printf("error = %lf\n", error);
		printcorrect();
		if (backpropagatecnt == stepcnt) break;
	}
}

void laymake()//layer이라는 이차원 배열을 만든다. 한편, layer[i][j]는 i(0<=i<hidn+2)번째 층의 j번째 뉴런의 값이다.
{
	for (int i = 0; i < hidn + 2; i++)
		layer[i] = (double*)malloc(sizeof(double)*layn[i]);
}

void weight_and_gradient_make()//weight_and_gradient_and_bestweight의 배열을 만든다.
{
	for (int i = 0; i < hidn + 1; i++)
	{
		weight[i] = (double**)malloc(sizeof(double*)*layn[i]);
		gradient[i] = (double**)malloc(sizeof(double*)*layn[i]);
		bestweight[i] = (double**)malloc(sizeof(double*)*layn[i]);
	}
	for (int i = 0; i < hidn+1; i++)
	{
		for (int j = 0; j < layn[i]; j++)
		{
			weight[i][j] = (double*)malloc(sizeof(double)*layn[i+1]);
			gradient[i][j] = (double*)malloc(sizeof(double)*layn[i+1]);
			bestweight[i][j] = (double*)malloc(sizeof(double)*layn[i + 1]);
		}
	}
}

double sigmoid(double x)//활성함수는 일단 sigmoid함수로 한다.
{
	return 1 / (1 + exp(-x));
}

void make_next_layer(int i)//layer[i]를 weight[i]들을 곱해 layer[i+1]을 만드는 과정
{
	for (int a = 0; a < layn[i+1]; a++)
	{
		double temp=0;
		for (int b = 0; b < layn[i]; b++)
		{
			temp += weight[i][b][a]*layer[i][b];
		}
		layer[i + 1][a] = sigmoid(temp);
	}
}

void make_all_layer()//모든 층을 만든다.
{
	for (int i = 0; i < hidn + 1;i++)
	{
		make_next_layer(i);
	}
}


void backpropagation(int n)//모든 gradient[i][j][k]=round(E)/round(weight[i][j][k])를 만든다.
{
	get_trainset(n);
	make_all_layer();
	for (int i = 0; i < layn[hidn]; i++)//일단 마지막 층의 gradient를 계산한다.
	{
		for (int j = 0; j < layn[hidn + 1]; j++)
		{
			gradient[hidn][i][j] = 2 * (layer[hidn + 1][j] - real[j])*layer[hidn + 1][j] * (1-layer[hidn + 1][j])*layer[hidn][i];
			weight[hidn][i][j] += -gradient[hidn][i][j] * rate;
		}
	}
	for (int i = hidn - 1; i >= 0; i--)//거슬러 올라가면서 gradient를 계산한다.
	{
		for (int j = 0; j < layn[i]; j++)
		{
			for (int k = 0; k < layn[i + 1]; k++)
			{
				double temp=0;
				for (int l = 0; l < layn[i + 2]; l++)
				{
					temp+=gradient[i + 1][k][l] * weight[i + 1][k][l];
				}
				gradient[i][j][k]=temp *layer[i][j] * (1 - layer[i + 1][k]);
				weight[i][j][k] += -gradient[i][j][k] * rate;
			}
		}
	}
}

void get_trainset(int n) {
	fseek(image, 16 + 784 * n, SEEK_SET);
	fseek(label, 8 + n, SEEK_SET);
	unsigned char data[784];
	fread(data, 784, 1, image);
	for (int i = 0; i < 28; i++) {
		for (int j = 1; j < 28; j++) {
			layer[0][i * 28 + j] = data[i * 28 + j];
			layer[0][i * 28 + j] /= (double)(255);
		}
	}
	for (int i = 0; i < layn[hidn+1]; i++) real[i] = 0;
	fread(data, 1, 1, label);
	real[data[0]] = 1;
}

void weight_initialization()//weight초기화 무작위로 초기화시킨다.
{
	for (int i = 0; i < hidn+1; i++)
	{
		for (int j = 0; j < layn[i]; j++)
		{
			for (int k = 0; k < layn[i + 1]; k++)
			{
				weight[i][j][k] = (double)rand()/100000;
			}
		}
	}
}

double geterror()//error을 얻는 함수
{
	error = 0;
	for (int i = 0; i < layn[hidn + 1]; i++)
	{
		error += (real[i]-layer[hidn+1][i])*(real[i] - layer[hidn + 1][i]);
	}
	return error;
}

int gettestcase(int n) {
	fseek(image2, 16 + 784 * n, SEEK_SET);
	fseek(label2, 8 + n, SEEK_SET);
	unsigned char data[784];
	fread(data, layn[0], 1, image2);
	for (int i = 0; i < 28; i++) {
		for (int j = 0; j < 28; j++) {
			layer[0][i * 28 + j] = data[i * 28 + j];
			layer[0][i * 28 + j] /= (double)(255);
		}
	}
	fread(data, 1, 1, label2);

	return data[0];
}
int predict() {
	int i, v;
	double big = -999;

	for (i = 0; i < layn[i] ; i++) {
		if (big < layer[hidn+1][i]) {
			big = layer[hidn + 1][i];
			v = i;
		}
	}
	return v;
}

void printcorrect() //문제가 발생한 시작 함수..
{
	int ans;
	double correct;
	int print[28][28] = {0};
	correct = 0;
	for (int i = 0; i<layn[hidn+1]; i++) for (int j = 0; j<layn[hidn+1]; j++) print[i][j] = 0;
	for (int i = 0; i<10000; i++) {
		ans = gettestcase(i);
		make_all_layer();
		print[ans][predict()]++;
		if (predict() == ans) correct++;
	}
	correct /= (double)(100);
	fprintf(out, "%lf\n", correct);
	if (bestcorrect <= correct) {
		bestcorrect = correct;
		for (int i = 0; i < hidn+1; i++)
		{
			for (int j = 0; j <layn[i]; i++) for (int k = 0; k < layn[i+1]; k++) bestweight[i][j][k] = weight[i][j][k];
		}
	}
	printf("%.2f%% 맞으셨습니다.\n", correct);
	for (int i = 0; i<layn[hidn+1]; i++) {
		for (int j = 0; j<layn[hidn+1]; j++) {
			printf("%5d ", print[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

int main()/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////메인함수에염
{
	laymake();
	weight_and_gradient_make();//이 다음에 입력을 해야한다. 그전은 기본적으로 필요한 것들이다.
	weight_initialization();//메인이 되려면 일단 여기까지는 무조건 있어야한다.
	setting();
	printcorrect();
	train(10);
	memoweight();
	memobestweight();
	printf("가장 좋은 값은 = %.2f%%입니다.\n", bestcorrect);
}

void getweight()
{
	int i;
	FILE *memoinput = fopen("weightmemo.txt", "rb");
	for (int j = 0; j < hidn; j++)
	{
		for (i = 0; i < layn[j] * layn[j + 1]; i++)
			fscanf(memoinput, "%lf", &weight[j][i / layn[j + 1]][i%layn[j+1]]);
	}
}


void memoweight()
{
	int i;
	FILE *memooutput = fopen("weightmemo.txt", "wb");
	for (int j = 0; j < hidn; j++)
	{
		for (i = 0; i < layn[j] * layn[j + 1]; i++)
			fprintf(memooutput, "%lf", weight[j][i / layn[j + 1]][i%layn[j + 1]]);
	}
}

void memobestweight()
{
	int i;
	FILE *bestweightoutput = fopen("bestweight.txt", "wb");
	for (int j = 0; j < hidn; j++)
	{
		for (i = 0; i < layn[j] * layn[j + 1]; i++)
			fprintf(bestweightoutput, "%lf", weight[j][i / layn[j + 1]][i%layn[j + 1]]);
	}
}
