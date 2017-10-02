#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<time.h>
#define hidn 1 //�������� ����: ���� ������ �Ѱ����� hidn+2�̴�.
#define rate 0.1//���״�� rate: gradient�� �󸶸� ���� ���ΰ�?
double**layer = (double**)malloc(sizeof(double*) * (hidn+2));//�� layer[0]�� �Է��� �� get�� �ǹ�, layer[hidn+1]�� ������� �ǹ��Ѵ�.
int layn[hidn + 2] = {784, 10, 10};//���ʷ�, �Է���������, hid���� ������, ������� ������
double***weight = (double***)malloc(sizeof(double**) * (hidn + 1));//weight[i][j][k]�� layer[i]���� layer[i+1]�� ���� ����ġ��, layer[i][j]�� layer[i][k]�� �̾��ִ� ����ġ�̴�.
double***gradient = (double***)malloc(sizeof(double**) * (hidn + 1));//round(E)/round(weight[i][j][k])�̴�.
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
void laymake();//layer���� ������ִ� ��.
void weight_and_gradient_make();//weight�� �����.
double sigmoid(double);//Ȱ���Լ��� �ϴ� sigmoid�Լ��� �Ѵ�.
void make_next_layer(int i);//layer[i]�� weight[i]���� ���� layer[i+1]�� ����� ����
void make_all_layer();//��� ���� �����.
void weight_initialization();//����ġ�� �������� �ʱ�ȭ�����ش�.
void backpropagation(int);//������
double geterror();//error�� ��� �Լ�
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
		printf("������ %d��° ������...\n", backpropagatecnt);
		for (i = 0; i<60000; i++) backpropagation(i);
		tmpcost = geterror();
		printf("error = %lf\n", error);
		printcorrect();
		if (backpropagatecnt == stepcnt) break;
	}
}

void laymake()//layer�̶�� ������ �迭�� �����. ����, layer[i][j]�� i(0<=i<hidn+2)��° ���� j��° ������ ���̴�.
{
	for (int i = 0; i < hidn + 2; i++)
		layer[i] = (double*)malloc(sizeof(double)*layn[i]);
}

void weight_and_gradient_make()//weight_and_gradient_and_bestweight�� �迭�� �����.
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

double sigmoid(double x)//Ȱ���Լ��� �ϴ� sigmoid�Լ��� �Ѵ�.
{
	return 1 / (1 + exp(-x));
}

void make_next_layer(int i)//layer[i]�� weight[i]���� ���� layer[i+1]�� ����� ����
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

void make_all_layer()//��� ���� �����.
{
	for (int i = 0; i < hidn + 1;i++)
	{
		make_next_layer(i);
	}
}


void backpropagation(int n)//��� gradient[i][j][k]=round(E)/round(weight[i][j][k])�� �����.
{
	get_trainset(n);
	make_all_layer();
	for (int i = 0; i < layn[hidn]; i++)//�ϴ� ������ ���� gradient�� ����Ѵ�.
	{
		for (int j = 0; j < layn[hidn + 1]; j++)
		{
			gradient[hidn][i][j] = 2 * (layer[hidn + 1][j] - real[j])*layer[hidn + 1][j] * (1-layer[hidn + 1][j])*layer[hidn][i];
			weight[hidn][i][j] += -gradient[hidn][i][j] * rate;
		}
	}
	for (int i = hidn - 1; i >= 0; i--)//�Ž��� �ö󰡸鼭 gradient�� ����Ѵ�.
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

void weight_initialization()//weight�ʱ�ȭ �������� �ʱ�ȭ��Ų��.
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

double geterror()//error�� ��� �Լ�
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

void printcorrect() //������ �߻��� ���� �Լ�..
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
	printf("%.2f%% �����̽��ϴ�.\n", correct);
	for (int i = 0; i<layn[hidn+1]; i++) {
		for (int j = 0; j<layn[hidn+1]; j++) {
			printf("%5d ", print[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

int main()/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////�����Լ�����
{
	laymake();
	weight_and_gradient_make();//�� ������ �Է��� �ؾ��Ѵ�. ������ �⺻������ �ʿ��� �͵��̴�.
	weight_initialization();//������ �Ƿ��� �ϴ� ��������� ������ �־���Ѵ�.
	setting();
	printcorrect();
	train(10);
	memoweight();
	memobestweight();
	printf("���� ���� ���� = %.2f%%�Դϴ�.\n", bestcorrect);
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
