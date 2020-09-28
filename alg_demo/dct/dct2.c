//==============================================================================
//  Copyright (C) 2020 zjz1988314. All rights reserved.
//
//  author:   zjz1988314
//  function: DCT2/IDCT2 math formular verify
//  time:     2020-09-28
//
//==============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.1416
//#define PI 3.1415926535
 
int MAT_SIZE;
 
float DCT_Mat[100][100]; //定于变换矩阵
float DctMap[100][100];  //输入矩阵，计算结束后为输出矩阵
float DctMapTmp[100][100];  //矩阵运算时用的中间矩阵

void InitTransMat()
{
    int i,j;
    float a;

    for(i=0;i<MAT_SIZE;i++)
    {
        for(j=0;j<MAT_SIZE;j++)
        {
            a = 0;
            if(i==0)
            {
                a=sqrt((float)1/MAT_SIZE);
            }
            else
            {
                a=sqrt((float)2/MAT_SIZE);
            }
            DCT_Mat[i][j]= a*cos((j+0.5)*PI*i/MAT_SIZE); //变换矩阵
        }
    }
    /* only for 8x8 */
    //float Tmp[100][100] = {
    //    {0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,},
    //    {0.4904,    0.4157,    0.2778,    0.0975,   -0.0975,   -0.2778,   -0.4157,   -0.4904,},
    //    {0.4619,    0.1913,   -0.1913,   -0.4619,   -0.4619,   -0.1913,    0.1913,    0.4619,},
    //    {0.4157,   -0.0975,   -0.4904,   -0.2778,    0.2778,    0.4904,    0.0975,   -0.4157,},
    //    {0.3536,   -0.3536,   -0.3536,    0.3536,    0.3536,   -0.3536,   -0.3536,    0.3536,},
    //    {0.2778,   -0.4904,    0.0975,    0.4157,   -0.4157,   -0.0975,    0.4904,   -0.2778,},
    //    {0.1913,   -0.4619,    0.4619,   -0.1913,   -0.1913,    0.4619,   -0.4619,    0.1913,},
    //    {0.0975,   -0.2778,    0.4157,   -0.4904,    0.4904,   -0.4157,    0.2778,   -0.0975,},
    //};
    //for (int i=0; i<8; i++)
    //    for (int j=0; j<8; j++)
    //        DCT_Mat[i][j] = Tmp[i][j];
}
 
void DCT2()
{
    float t=0;
    int i,j,k;
    for(i=0;i<MAT_SIZE;i++)  //相当于A*I
    {
        for(j=0;j<MAT_SIZE;j++)
        {
            t=0;
            for(k=0;k<MAT_SIZE;k++)
            {
                t+=DCT_Mat[i][k]*DctMap[k][j]; //矩阵的乘法，DCT_Mat的第i行乘DctMap的第j列
            }
            DctMapTmp[i][j]=t;
        }
    }
    for(i=0;i<MAT_SIZE;i++)  //相当于（A*I）后再*A‘
    {
        for(j=0;j<MAT_SIZE;j++)
        {
            t=0;
            for(k=0;k<MAT_SIZE;k++)
            {
                t+=DctMapTmp[i][k]*DCT_Mat[j][k];
            }
            DctMap[i][j]=t;
        }
    }
}

void IDCT2()
{
    float t=0;
    int i,j,k;
    for(i=0;i<MAT_SIZE;i++)  //相当于A'*I
    {
        for(j=0;j<MAT_SIZE;j++)
        {
            t=0;
            for(k=0;k<MAT_SIZE;k++)
            {
                t+=DCT_Mat[k][i]*DctMap[k][j]; //矩阵的乘法，DCT_Mat的第i列乘DctMap的第j列
            }
            DctMapTmp[i][j]=t;
        }
    }
    for(i=0;i<MAT_SIZE;i++)  //相当于（A*I）后再*A‘
    {
        for(j=0;j<MAT_SIZE;j++)
        {
            t=0;
            for(k=0;k<MAT_SIZE;k++)
            {
                t+=DctMapTmp[i][k]*DCT_Mat[k][j];
            }
            DctMap[i][j]=t;
        }
    }
}
 
void InitDctMap()
{
    float TmpDctMap[100][100] = {
        {84,    23,    40,    72,    70,    67,     5,    80},
        {37,    55,    41,    51,    48,   100,    57,     8},
        {62,    93,    66,    78,    11,    96,    70,    95},
        {73,    34,    84,    49,    66,     6,    96,    92},
        {19,    66,    37,    19,    37,    36,    75,    60},
        {90,    39,    43,    70,    14,    55,    74,    25},
        {57,    63,    59,    98,    57,    26,    43,    87},
        {63,    70,    57,    81,    82,    60,    63,    51},
    };
    for (int i=0; i<MAT_SIZE; i++)
        for (int j=0; j<MAT_SIZE; j++)
            DctMap[i][j] = TmpDctMap[i][j];
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        puts("error input for DCT2/IDCT2 test!!! ");
        puts("usage: cmd n(n in [1,100])");
        return -1;
    }
    puts("-------------------DCT2/IDCT2 bgein--------------------");

    MAT_SIZE = atoi(argv[1]); //定义矩阵维度
    InitDctMap();
    InitTransMat();

    puts("-----------------raw matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
        for(int j=0;j<MAT_SIZE;j++)
        {
            printf("%0.2f\t", DctMap[i][j]); //输出DCT变换结果
        }
        puts("");
    }

    DCT2();
    puts("-----------------dct2 matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
        for(int j=0;j<MAT_SIZE;j++)
        {
            printf("%0.2f\t", DctMap[i][j]); //输出DCT变换结果
        }
        puts("");
    }

    IDCT2();
    puts("-----------------idct2 matrix--------------------");
    for(int i=0;i<MAT_SIZE;i++)
    {
        for(int j=0;j<MAT_SIZE;j++)
        {
            printf("%0.2f\t", DctMap[i][j]); //输出DCT变换结果
        }
        puts("");
    }
    puts("-------------------------------------------------");

    puts("-------------------DCT2/IDCT2 end--------------------");

    return 0;
}
