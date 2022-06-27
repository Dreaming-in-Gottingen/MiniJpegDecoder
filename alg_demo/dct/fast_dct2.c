//==============================================================================
//  Copyright (C) 2022 zjz1988314. All rights reserved.
//
//  author:      zjz1988314
//  description: Fast dct2 to repace standard DCT2/IDCT2 to accelerate calc,
//               it mainly test 4x4 dct2, which is used in h.264.
//               But fast dct2 is an approximation to the 4x4 DCT.
//  time:        2022-06-27
//
//==============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int MAT_SIZE = 4;

int DctMap[4][4];               // input raw data that need to be transformed
float DctRes[4][4];             // output data that transformed
float IDctRes[4][4];            // output data that inverse transformed

int DctCore[4][4];              // 2D transform core
float DctScaleFactor[4][4];     // used to scale in DCT

float IDctCore[4][4];             // 2D inverse transform core
float IDctScaleFactor[4][4];    // used to pre scale in IDCT

void InitDctMap()
{
    int pic[4][4] = {
        { 5, 11,  8, 10},
        { 9,  8,  4, 12},
        { 1, 10, 11,  4},
        {19,  6, 15,  7},
    };

    for (int i=0; i<MAT_SIZE; i++) {
        for (int j=0; j<MAT_SIZE; j++) {
            DctMap[i][j] = pic[i][j];
        }
    }
}

void InitTransMat()
{
    // DCT
    int core[4][4] = {
        {1,  1,  1,  1},
        {2,  1, -1, -2},
        {1, -1, -1,  1},
        {1, -2,  2, -1},
    };
    for (int i=0; i<MAT_SIZE; i++) {
        for (int j=0; j<MAT_SIZE; j++) {
            DctCore[i][j] = core[i][j];
        }
    }

    float a = 1.0/2;
    float b = sqrt(2.0/5);

    float scale[4][4] = {
        {a*a,   a*b/2, a*a,   a*b/2},
        {a*b/2, b*b/4, a*b/2, b*b/4},
        {a*a,   a*b/2, a*a,   a*b/2},
        {a*b/2, b*b/4, a*b/2, b*b/4},
    };
    for (int i=0; i<MAT_SIZE; i++) {
        for (int j=0; j<MAT_SIZE; j++) {
            DctScaleFactor[i][j] = scale[i][j];
        }
    }

    // IDCT
    float icore[4][4] = {
        {1,    1,   1,  0.5},
        {1,  0.5,  -1,   -1},
        {1, -0.5,  -1,    1},
        {1,   -1,   1, -0.5},
    };
    for (int i=0; i<MAT_SIZE; i++) {
        for (int j=0; j<MAT_SIZE; j++) {
            IDctCore[i][j] = icore[i][j];
        }
    }
    float iscale[4][4] = {
        {a*a, a*b, a*a, a*b},
        {a*b, b*b, a*b, b*b},
        {a*a, a*b, a*a, a*b},
        {a*b, b*b, a*b, b*b},
    };
    for (int i=0; i<MAT_SIZE; i++) {
        for (int j=0; j<MAT_SIZE; j++) {
            IDctScaleFactor[i][j] = iscale[i][j];
        }
    }
}

// transform formular: (C X C^) * E
// C: DctCore
// X: DctMap
// E: DctScaleFactor
void DCT2()
{
    int t = 0;
    int i,j,k;
    int DctMapTmp1[4][4];  // tmp data
    int DctMapTmp2[4][4];  // tmp data

#if 0
    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            t = 0;
            for (k=0; k<MAT_SIZE; k++)
            {
                t += DctCore[i][k] * DctMap[k][j];
            }
            DctMapTmp1[i][j] = t;
        }
    }

    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            t = 0;
            for (k=0; k<MAT_SIZE; k++)
            {
                t += DctMapTmp1[i][k] * DctCore[j][k];
            }
            DctMapTmp2[i][j] = t;
        }
    }
#else
    for (j=0; j<MAT_SIZE; j++)
        DctMapTmp1[0][j] = DctMap[0][j] + DctMap[1][j] + DctMap[2][j] + DctMap[3][j];
    for (j=0; j<MAT_SIZE; j++)
        DctMapTmp1[1][j] = (DctMap[0][j]<<1) + DctMap[1][j] - DctMap[2][j] - (DctMap[3][j]<<1);
    for (j=0; j<MAT_SIZE; j++)
        DctMapTmp1[2][j] = DctMap[0][j] - DctMap[1][j] - DctMap[2][j] + DctMap[3][j];
    for (j=0; j<MAT_SIZE; j++)
        DctMapTmp1[3][j] = DctMap[0][j] - (DctMap[1][j]<<1) + (DctMap[2][j]<<1) - DctMap[3][j];

    for (i=0; i<MAT_SIZE; i++)
        DctMapTmp2[i][0] = DctMapTmp1[i][0] + DctMapTmp1[i][1] + DctMapTmp1[i][2] + DctMapTmp1[i][3];
    for (i=0; i<MAT_SIZE; i++)
        DctMapTmp2[i][1] = (DctMapTmp1[i][0]<<1) + DctMapTmp1[i][1] - DctMapTmp1[i][2] - (DctMapTmp1[i][3]<<1);
    for (i=0; i<MAT_SIZE; i++)
        DctMapTmp2[i][2] = DctMapTmp1[i][0] - DctMapTmp1[i][1] - DctMapTmp1[i][2] + DctMapTmp1[i][3];
    for (i=0; i<MAT_SIZE; i++)
        DctMapTmp2[i][3] = DctMapTmp1[i][0] - (DctMapTmp1[i][1]<<1) + (DctMapTmp1[i][2]<<1) - DctMapTmp1[i][3];
#endif

    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            DctRes[i][j] = DctMapTmp2[i][j] * DctScaleFactor[i][j];
        }
    }
}

// inverse transform: C^ (Y * E) C
void IDCT2()
{
    float t=0;
    int i,j,k;
    float IDctMapTmp1[4][4];  // tmp data
    float IDctMapTmp2[4][4];  // tmp data

    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            IDctMapTmp1[i][j] = DctRes[i][j] * IDctScaleFactor[i][j];
        }
    }

#if 0
    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            t=0;
            for (k=0; k<MAT_SIZE; k++)
            {
                t += IDctCore[i][k] * IDctMapTmp1[k][j];
            }
            IDctMapTmp2[i][j]=t;
        }
    }
    for (i=0; i<MAT_SIZE; i++)
    {
        for (j=0; j<MAT_SIZE; j++)
        {
            t=0;
            for (k=0; k<MAT_SIZE; k++)
            {
                t += IDctMapTmp2[i][k] * IDctCore[j][k];
            }
            IDctRes[i][j] = t;
        }
    }
#else
    for (j=0; j<MAT_SIZE; j++)
        IDctMapTmp2[0][j] = IDctMapTmp1[0][j] + IDctMapTmp1[1][j] + IDctMapTmp1[2][j] + (IDctMapTmp1[3][j]/2);
    for (j=0; j<MAT_SIZE; j++)
        IDctMapTmp2[1][j] = IDctMapTmp1[0][j] + (IDctMapTmp1[1][j]/2) - IDctMapTmp1[2][j] - IDctMapTmp1[3][j];
    for (j=0; j<MAT_SIZE; j++)
        IDctMapTmp2[2][j] = IDctMapTmp1[0][j] - (IDctMapTmp1[1][j]/2) - IDctMapTmp1[2][j] + IDctMapTmp1[3][j];
    for (j=0; j<MAT_SIZE; j++)
        IDctMapTmp2[3][j] = IDctMapTmp1[0][j] - IDctMapTmp1[1][j] + IDctMapTmp1[2][j] - (IDctMapTmp1[3][j]/2);

    for (i=0; i<MAT_SIZE; i++)
        IDctRes[i][0] = IDctMapTmp2[i][0] + IDctMapTmp2[i][1] + IDctMapTmp2[i][2] + (IDctMapTmp2[i][3]/2);
    for (i=0; i<MAT_SIZE; i++)
        IDctRes[i][1] = IDctMapTmp2[i][0] + (IDctMapTmp2[i][1]/2) - IDctMapTmp2[i][2] - IDctMapTmp2[i][3];
    for (i=0; i<MAT_SIZE; i++)
        IDctRes[i][2] = IDctMapTmp2[i][0] - (IDctMapTmp2[i][1]/2) - IDctMapTmp2[i][2] + IDctMapTmp2[i][3];
    for (i=0; i<MAT_SIZE; i++)
        IDctRes[i][3] = IDctMapTmp2[i][0] - IDctMapTmp2[i][1] + IDctMapTmp2[i][2] - (IDctMapTmp2[i][3]/2);
#endif
}

int main(int argc, char *argv[])
{
    puts("-------------------4x4 DCT2/IDCT2 bgein--------------------");

    InitDctMap();
    InitTransMat();

    puts("-----------------raw matrix--------------------");
    for (int i=0; i<MAT_SIZE; i++)
    {
        for (int j=0; j<MAT_SIZE; j++)
        {
            printf("%d\t", DctMap[i][j]);
        }
        puts("");
    }

    DCT2();
    puts("-----------------dct2 matrix--------------------");
    for (int i=0; i<MAT_SIZE; i++)
    {
        for (int j=0; j<MAT_SIZE; j++)
        {
            printf("%0.2f\t", DctRes[i][j]);
        }
        puts("");
    }

    IDCT2();
    puts("-----------------idct2 matrix--------------------");
    for (int i=0; i<MAT_SIZE; i++)
    {
        for (int j=0; j<MAT_SIZE; j++)
        {
            printf("%0.2f\t", IDctRes[i][j]);
        }
        puts("");
    }
    puts("-------------------------------------------------");

    puts("-------------------4x4 DCT2/IDCT2 end--------------------");

    return 0;
}
