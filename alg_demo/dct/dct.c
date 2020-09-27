//==============================================================================
//  Copyright (C) 2020 zjz1988314. All rights reserved.
//
//  靠: zjz1988314
//  靠: 靠DCT/IDCT靠
//  靠: 2020-09-28
//
//==============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.1416
//#define PI 3.1415926535
 
int MAT_SIZE;
 
float DctMap[100];     //输入矩阵，计算结束后为输出矩阵
float DctMapRes[100];  //一维DCT结果
float IDctMapRes[100]; //一维IDCT结果

void InitDctMap()
{
    float Tmp[100] = {50, 55, 67, 80, -10, -5, 20, 30};
    for (int i=0; i<MAT_SIZE; i++)
        DctMap[i] = Tmp[i];
}
 
void DCT()
{
    float t, c;
    int i,j;

    for(i=0;i<MAT_SIZE;i++)
	{
        if (i==0)
            c = sqrt(1.0/MAT_SIZE);
        else
            c = sqrt(2.0/MAT_SIZE);

        t=0;
        for(j=0;j<MAT_SIZE;j++)
		{
            t += c*DctMap[j]*cos(PI*i*(j+0.5)/MAT_SIZE); //核心算法
        }
        DctMapRes[i] = t;
    }
}

void IDCT()
{
    float t, c;
    int i,j;

    for(i=0;i<MAT_SIZE;i++)
	{
        t=0;
        for(j=0;j<MAT_SIZE;j++)
		{
            c = (j==0) ? sqrt(1.0/MAT_SIZE): sqrt(2.0/MAT_SIZE);
            t += c*DctMapRes[j]*cos(PI*j*(i+0.5)/MAT_SIZE); //核心算法
        }
        IDctMapRes[i] = t;
    }
}
 
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        puts("error input for DCT/IDCT test!!! ");
        puts("usage: cmd n(n<=8)");
        return -1;
    }
    puts("-------------------DCT/IDCT bgein--------------------");

	MAT_SIZE = atoi(argv[1]); //定义矩阵维度

    InitDctMap(); //初始化输入向量

    puts("-------------------raw matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
        printf("%f\t", DctMap[i]); //输出DCT变换前数据
    }

    DCT();
    puts("\n-----------------dct matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
            printf("%f\t", DctMapRes[i]); //输出DCT变换结果
    }

    IDCT();
    puts("\n-----------------idct matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
            printf("%f\t", IDctMapRes[i]); //输出IDCT变换结果
    }
    puts("\n-------------------------------------------------");

    puts("-------------------DCT/IDCT end--------------------");

    return 0;
}
