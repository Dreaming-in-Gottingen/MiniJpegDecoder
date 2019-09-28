#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "AString.h"
#include "ADebug.h"
#include "ABitReader.h"
#include "DataSource.h"
#include "FileSource.h"

using namespace std;
using namespace codec_utils;


//below for jpeg marker
#define SOI 0xFFD8      //Start of image
#define EOI 0xFFD9      //End of image

#define APP0 0xFFE0     //Application data segment
#define DQT 0xFFDB      //Define quantization table(s)
#define SOF 0xFFC0      //Baseline DCT
#define DHT 0xFFC4      //Define Huffman table(s)
#define SOS 0xFFDA      //Start of scan


struct jpegParam {

    //DQT
    uint8_t *pQT[4];

    //DHT
    uint8_t *pHTCodeCnt[2][2];      //count for x bits, read from DHT, array len:16
    uint16_t HTCodeRealCnt[2][2];   //sum of *pHTCodeCnt[i][j], total count of *pHTCodeWidth[i][j] or pHTCode[i][j]
    uint8_t *pHTCodeWidth[2][2];    //2-16 bits width, real array len: sum(pHTCodeCnt), max array len: (2^2 - 1) + (2^3 - 1) + (2^4 - 1) + ... + (2^16 - 1)
    uint16_t *pHTCode[2][2];        //map by *pHTCodeWidth[i][j]
    uint8_t *pHTCodeVal[2][2];      //follow by CodeCnt, len=sum(pHTCodeCnt)

    //SOF
    uint32_t width;
    uint32_t height;
    uint8_t bit_width;      //Sample precision
    uint8_t comp_cnt;       //Number of image components in frame, same as SOS's
    uint8_t qt_comp_id[3];  //Component identifier, 1-Y, 2-Cb, 3-Cr
    uint8_t hfactor[3];     //Horizontal sampling factor
    uint8_t vfactor[3];     //Vertical sampling factor
    uint8_t qt_id[3];       //Quantization table destination selector

    //SOS
    uint8_t ht_comp_id[3];  //1-Y, 2-Cb, 3-Cr
    uint8_t ht_idx[3];      //0xAB -> A: DC huffman table idx; B: AC huffman table idx.

    //huffman decode intermediate values
    int  cur_type;          //0-Y; 1-Cb; 2-Cr
    bool YDC_dpcm;          //0-first block; 1-others
    int  YDC_last;
    bool CbDC_dpcm;
    int  CbDC_last;
    bool CrDC_dpcm;
    int  CrDC_last;
};


int parseApp0(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    abr->skipBits(len*8 - 16);

    return 0;
}

int parseDQT(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    int idx = abr->getBits(8) & 0x0f;
    param->pQT[idx] = (uint8_t*)malloc(64);
    uint8_t *ptr = param->pQT[idx];
    memcpy(param->pQT[idx], abr->data(), 64);
    abr->skipBits(64*8);

    return 0;
}

//rebuild huffman table
int parseDHT(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    uint8_t idx = abr->getBits(8);
    uint8_t idx_high = idx>>4;
    uint8_t idx_low = idx & 0x0f;

    //generate pHTCodeCnt[idx_high][idx_low]
    uint8_t *pCodeCnt = (uint8_t*)malloc(16);
    int i, j;
    int total_code_cnt = 0;
    printf("\tCodeCnt:\t");
    for (i=0; i<16; i++) {
        int code_cnt = abr->getBits(8);
        pCodeCnt[i] = code_cnt;
        total_code_cnt += code_cnt;
        printf("%2d  ", code_cnt);
    }
    //printf("\n\ttotal code cnt: %d\n", total_code_cnt);
    param->HTCodeRealCnt[idx_high][idx_low] = total_code_cnt;
    param->pHTCodeCnt[idx_high][idx_low] = pCodeCnt;

    uint8_t *pWidth = (uint8_t *)malloc(total_code_cnt);
    param->pHTCodeWidth[idx_high][idx_low] = pWidth;
    printf("\n\tCodeWidth:\t");
    for (i=0, j=0; i<16; i++, j=0) {
        while (j++ < pCodeCnt[i]) {
            uint8_t tmp = *pWidth++ = i+1;
            printf("%2d  ", tmp);
        }
    }
    puts("");

    pWidth = param->pHTCodeWidth[idx_high][idx_low];

    //generate pHTCode[idx_high][idx_low]
    uint16_t *pCode = (uint16_t*)malloc(2*total_code_cnt);   //huffman code width: 2~16 bits
    param->pHTCode[idx_high][idx_low] = pCode;
    uint16_t last_code = 0;
    for (i=0; i<16; i++) {
        int j = 0;
        uint16_t tmp;
        while (j++ < pCodeCnt[i]) {
            if ((i==1) && (j==1)) {
                *pCode = 0;
                //printf("a: %d\n", *pCode);
            } else if (j == 1) {        //first add x bits
                int k = i;
                int shift_bits = 1;
                while(pCodeCnt[--k] == 0) {
                    shift_bits++;
                }
                tmp = (*pCode+1)<<shift_bits;
                *++pCode = tmp;
                //printf("b: %d\n", *pCode);
            } else {
                tmp = *pCode + 1;
                *++pCode = tmp;
                //printf("c: %d\n", *pCode);
            }
            //printf("i:%d, j:%d, (%d , %d)=> %#x\n", i, j, pCodeCnt[i], pWidth[i], *pCode);
        }
    }

    //generate pHTCodeVal[idx_high][idx_low]
    uint8_t *pCodeVal = (uint8_t*)malloc(total_code_cnt);   //huffman code width: 2~16 bits
    param->pHTCodeVal[idx_high][idx_low] = pCodeVal;
    for (i=0; i<total_code_cnt; i++) {
        *pCodeVal++ = abr->getBits(8);
    }

    printf("\t-----------------huffman table: [%d][%d]----[ith] (CodeWidth, Code) -> CodeVal-----------------\n", idx_high, idx_low);

    pWidth = param->pHTCodeWidth[idx_high][idx_low];
    pCode = param->pHTCode[idx_high][idx_low];
    pCodeVal = param->pHTCodeVal[idx_high][idx_low];
    for (i=0; i<total_code_cnt; i++) {
        printf("\t[%2d] (%2d, %#x) -> %#x\n", i, *pWidth++, *pCode++, *pCodeVal++);
    }
    puts("\t---------------------------------------------------------------------------------------------");

    return 0;
}

//map comp_id to quantized_table
int parseSOF(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    param->bit_width = abr->getBits(8);
    param->height = abr->getBits(16);
    param->width = abr->getBits(16);
    printf("\tres=> (%d x %d)\n", param->width, param->height);
    param->comp_cnt = abr->getBits(8);
    int cnt = 0;
    while (cnt < param->comp_cnt) {
        param->qt_comp_id[cnt] = abr->getBits(8);
        uint8_t factor = abr->getBits(8);
        param->hfactor[cnt] = factor>>4;
        param->vfactor[cnt] = factor & 0x0f;
        param->qt_id[cnt] = abr->getBits(8);
        printf("\tsampling_factor=> comp_id:[%d], qt_id:[%d], factor:(%d x %d)\n", param->qt_comp_id[cnt], param->qt_id[cnt], param->hfactor[cnt], param->vfactor[cnt]);
        cnt++;
    }

    return 0;
}

//map comp_id to huffman_table
int parseSOS(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    int comp_cnt = abr->getBits(8);
    int i;
    for (i=0; i<comp_cnt; i++) {
        //abr->skipBits(8);
        CHECK_EQ(abr->getBits(8), i+1);
        int ht_idx = abr->getBits(8);
        param->ht_comp_id[i] = ht_idx;
        //int dc_idx = ht_idx>>4;
        //int ac_idx = ht_idx & 0x0f;
    }
    uint32_t baseline_flag = abr->getBits(24);
    puts("\tonly support baseline profile!");
    CHECK_EQ(baseline_flag, 0x003f00);

    return 0;
}

int ParseJpegHeader(FileSource *fs, struct jpegParam *param)
{
    bool exit_flag = false;
    int offset = -1;

    uint8_t *buf = (uint8_t*)malloc(4096);
    fs->readAt(0, buf, 4096);
    struct ABitReader abr(buf, 4096);

    uint16_t jpeg_tag = abr.getBits(16);
    if (jpeg_tag == SOI) {
        puts("jpeg file begin!");
    } else {
        printf("not jpeg file with begin bytes(%#x)\n", jpeg_tag);
        goto exit;
    }

    while(exit_flag == false) {
        jpeg_tag = abr.getBits(16);
        switch (jpeg_tag) {
            case APP0:
                parseApp0(&abr, param);
                break;
            case DQT:
                parseDQT(&abr, param);
                break;
            case SOF:
                parseSOF(&abr, param);
                break;
            case DHT:
                parseDHT(&abr, param);
                break;
            case SOS:
                parseSOS(&abr, param);
                exit_flag = true;
                break;
            case EOI:
                break;
            default:
                CHECK(0);
                break;
        }
    }
    offset = abr.getOffset();

exit:
    free(buf);
    return offset;
}

//color: Y or CbCr -> 0:Y,  1:CbCr
//idx:   DC or AC  -> 0:DC, 1:AC
//[0][0] Y0,     [1][0] Y[1-63]
//[0][1] Cb0/Cr0, [1][1] Cb/Cr[1-63]
//RLC(A,B) <-> (*freq, *coef)
//ret: 1-EOB, 0-none_EOB
int HuffmanDecode(struct ABitReader *abr, struct jpegParam *param, int color, int idx, int *freq, int *coef)
{
    uint8_t *pCodeCnt = param->pHTCodeCnt[idx][color];
    uint16_t *pCode = param->pHTCode[idx][color];
    uint8_t *pCodeVal = param->pHTCodeVal[idx][color];

    int i = 1;
    while (i < 16 ) {
        if (pCodeCnt[i] != 0) {
            uint16_t code=abr->getBits(i+1);
            bool find = true;
            int cnt = 0;
            while (*pCode++ != code) {
                pCodeVal++;
                if (++cnt == pCodeCnt[i]) {
                    find = false;
                    abr->putBits(code, i+1);
                    //printf("  discard code-bit-width:%d, for we have foreached to the end:%d\n", i+1, cnt);
                    break;
                }
            }
            if (find) {
                int code_val = abr->getBits(*pCodeVal & 0x0f);
                //printf("    get the right code(%#x)! bit_width:%d -> %#x -> %#x\n", code, i+1, *pCodeVal, code_val);
                if (idx == 0) {        //DC
                    CHECK((*pCodeVal & 0xf0) == 0);
                    *freq= 0;
                    int coef_bw = *pCodeVal & 0x0f;
                    if (code_val >> (coef_bw-1)) {    //  MSB(Most Significant Bit) = 1, positive value
                        *coef = code_val;
                    } else {                // MSB = 0
                        uint32_t tmp1 = (0xffffffff>>coef_bw)<<coef_bw | code_val;
                        uint32_t tmp2 = ~tmp1;
                        *coef = -tmp2;
                        //printf("------(%#x, %#x, %d, %#x)--------\n", tmp1, tmp2, *coef, code_val);
                    }
                    return 0;
                } else if (idx == 1){ //AC
                    *freq = *pCodeVal >> 4;
                    if (*pCodeVal == 0xf0) {
                    //if (code_val==0xf0) {
                        *freq = 16;
                        *coef = 0;
                        return 0;
                    } else if (*pCodeVal == 0x00) { // EOB: all of below coefficient is 0
                    //} else if (code_val==0x00) { // EOB: all of below coefficient is 0
                        *freq = 0; //none sense
                        *coef = 0;
                        return 1;  //careful for this ret, it means EOB!
                    } else {
                        int coef_bw = *pCodeVal & 0x0f;
                        if (code_val >> (coef_bw-1)) {    //  MSB(Most Significant Bit) = 1
                            *coef = code_val;
                        } else {                // MSB = 0
                            //uint32_t tmp1 = 0xffffffff & code_val;
                            //uint32_t tmp2 = ~tmp1;
                            //*coef = tmp2;
                            uint32_t tmp1 = (0xffffffff>>coef_bw)<<coef_bw | code_val;
                            uint32_t tmp2 = ~tmp1;
                            *coef = -tmp2;
                            //printf("------(%#x, %#x, %d, %#x)--------\n", tmp1, tmp2, *coef, code_val);
                        }
                        return 0;
                    }
                } else {
                    TRESPASS();
                }
                break;
            } // if (find)
        } // if (pCodeCnt[i] != 0)
        i++;
    }
    TRESPASS();

    return 0;
}

//color: 0-Y; 1-CbCr
int RebuildMCU1X1(struct ABitReader *abr, struct jpegParam *param, int color, int *block)
{
    int *ptr = block;
    memset(ptr, 0, 8*8*4);

    int cnt, coef;

    //DC
    int ret = HuffmanDecode(abr, param, color, 0, &cnt, &coef);
    int extra;
    switch (param->cur_type) {
        case 0:
            extra = param->YDC_dpcm==0? 0:param->YDC_last;
            coef +=extra;
            param->YDC_last = coef;
            break;
        case 1:
            extra = param->CbDC_dpcm==0? 0:param->CbDC_last;
            coef +=extra;
            param->CbDC_last = coef;
            break;
        case 2:
            extra = param->CrDC_dpcm==0? 0:param->CrDC_last;
            coef +=extra;
            param->CrDC_last = coef;
            break;
        default:
            TRESPASS();
            break;
    }
    *ptr++ = coef;
    //printf("(%d)\n", coef);

    //AC
    int i;
    for (i=1; i<64; i++) {
        ret = HuffmanDecode(abr, param, color, 1, &cnt, &coef);
        //printf("(%d, %d)\n", cnt, coef);
        while(cnt--) {
            *ptr++ = 0;
        }
        *ptr++ = coef;

        if (ret == 1) {
            break;
        }
    }
    if (i==64) {
        puts("Oops! somethings go wrong!");
        TRESPASS();
    }

    //dump block
    for (i=0; i<8; i++) {
        for (int j=0; j<8; j++) {
            printf("%4d  ", *block++);
        }
        puts("");
    }
    puts("");

    return 0;
}

// IRLC + IZigzag + IDPCM + ISQ + IDCT
int JpegDecode(FileSource *fs, struct jpegParam *param, int offset)
{
    int width, height;
    int block[8][8];
    memset(&block, 0, 64*4);

    off64_t file_sz;
    fs->getSize(&file_sz);
    uint8_t *buf = (uint8_t*)malloc(file_sz);
    fs->readAt(offset, buf, file_sz);
    struct ABitReader abr(buf, file_sz);

    int i,j;
    for (i=0; i<20; i++) {
        printf("------------------block(0,%d)----------------------\n", i);
        param->cur_type = 0;
        param->YDC_dpcm = i;
        RebuildMCU1X1(&abr, param, 0, &block[0][0]);
        param->cur_type = 1;
        param->CbDC_dpcm = i;
        RebuildMCU1X1(&abr, param, 1, &block[0][0]);
        param->cur_type = 2;
        param->CrDC_dpcm = i;
        RebuildMCU1X1(&abr, param, 1, &block[0][0]);
    }

    free(buf);

    return 0;
}

int main(void)
{
    cout<<"------begin-------"<<endl;
    FileSource *pFS = new FileSource("testrgb-1x1.jpg");
    struct jpegParam param;
    memset(&param, 0, sizeof(param));

    off64_t sz;
    pFS->getSize(&sz);
    cout<<"sz: "<<sz<<endl;

    int offset = ParseJpegHeader(pFS, &param);
    if (offset<0) {
        puts("parse jpeg header failed!");
    } else {
        JpegDecode(pFS, &param, offset);
    }

    delete pFS;
    cout<<"------end-------"<<endl;

    return 0;
}
