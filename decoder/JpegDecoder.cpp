#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
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

#define DRI 0xFFDD      //Define Restart Interval

enum {
    EOB = 1,
};

enum COLOR_TYPE {
    COLOR_Y = 0,
    COLOR_UV = 1,
};

enum COEF_TYPE {
    COEF_DC = 0,
    COEF_AC = 1,
};

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

    uint32_t row_stride;    //block_cnt*8, maybe larger than width

    //DRI
    uint32_t restart_interval;  //count for MCU restart scan, will meet RSTn(FF D0-D7) when restart MCU scan
    uint32_t scan_mcu_cnt;      //increase until restart_interval

    //SOS
    uint8_t ht_comp_id[3];  //1-Y, 2-Cb, 3-Cr
    uint8_t ht_idx[3];      //0xAB -> A: DC huffman table idx; B: AC huffman table idx.

    //huffman decode intermediate values
    int  cur_type;          //0-Y; 1-Cb; 2-Cr
    //bool YDC_dpcm;          //0-first block; 1-others
    int  YDC_last;          //init_val=0, cur_val = coef + last_val
    int  CbDC_last;
    int  CrDC_last;

    int (*RebuildMCUFunc)(struct ABitReader *abr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*dst)[8], int i, int j, bool dump);

    uint8_t *py_data;
    uint8_t *pu_data;
    uint8_t *pv_data;
};

static const uint8_t zigzag[64] =
{
   0,  1,  5,  6, 14, 15, 27, 28,
   2,  4,  7, 13, 16, 26, 29, 42,
   3,  8, 12, 17, 25, 30, 41, 43,
   9, 11, 18, 24, 31, 40, 44, 53,
  10, 19, 23, 32, 39, 45, 52, 54,
  20, 22, 33, 38, 46, 51, 55, 60,
  21, 34, 37, 47, 50, 56, 59, 61,
  35, 36, 48, 49, 57, 58, 62, 63
};

int parseApp0(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), App0 offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    abr->skipBits(len*8 - 16);

    return 0;
}

int parseDQT(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), DQT offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    len -= 2;
    while (len>0)
    {
        int idx = abr->getBits(8) & 0x0f;
        param->pQT[idx] = (uint8_t*)malloc(64);
        memcpy(param->pQT[idx], abr->data(), 64);
        abr->skipBits(64*8);

        int i,j;
        uint8_t *ptr = param->pQT[idx];
        printf("\tQuantization table for color_id(%d):\t", idx);
        for (i=0; i<8; i++) {
            printf("\n\t");
            for (j=0; j<8; j++) {
                printf("%2d  ", *ptr++);
            }
        }

        puts("\n\t---------------------------------------------------------------------------------------------");
        len -= 65;
    }

    return 0;
}

//rebuild huffman table
int parseDHT(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), DHT offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    len -= 2;
    while (len>0)
    {
        uint8_t idx = abr->getBits(8);
        uint8_t idx_high = idx>>4;
        uint8_t idx_low = idx & 0x0f;

        //idx_hight represent DC or AC: 0-DC, 1-AC
        //idx_low represent color id: 0-Y, 1-uv
        //[0][x] -- DC table, [0][0]:DC0, [0][1]:DC1
        //[1][x] -- AC table, [1][0]:AC0, [1][1]:AC1
        //generate pHTCodeCnt[idx_high][idx_low]
        uint8_t *pCodeCnt = (uint8_t*)malloc(16);
        int i, j;
        int total_code_cnt = 0;
        printf("\ttable id: [%d][%d]--[%s%d], dump more detail info...\n", idx_high, idx_low, idx_high==0?"DC":"AC", idx_low);
        printf("\tCodeCntOfNBits:\t");
        for (i=0; i<16; i++) {
            int code_cnt = abr->getBits(8);
            pCodeCnt[i] = code_cnt;
            total_code_cnt += code_cnt;
            printf("%2d  ", code_cnt);
        }
        printf("\n\ttotal code cnt: %d\n", total_code_cnt);
        param->HTCodeRealCnt[idx_high][idx_low] = total_code_cnt;
        param->pHTCodeCnt[idx_high][idx_low] = pCodeCnt;

        uint8_t *pWidth = (uint8_t *)malloc(total_code_cnt);
        param->pHTCodeWidth[idx_high][idx_low] = pWidth;
        printf("\tValidCodeWidth:\t");
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
                } else if (j == 1) {        //first add x bits
                    int k = i;
                    int shift_bits = 1;
                    while(pCodeCnt[--k] == 0) {
                        shift_bits++;
                    }
                    tmp = (*pCode+1)<<shift_bits;
                    *++pCode = tmp;
                } else {
                    tmp = *pCode + 1;
                    *++pCode = tmp;
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

        printf("\t-----------------huffman table: [%d][%d]---------------------\n", idx_high, idx_low);

        pWidth = param->pHTCodeWidth[idx_high][idx_low];
        pCode = param->pHTCode[idx_high][idx_low];
        pCodeVal = param->pHTCodeVal[idx_high][idx_low];
        puts("\t[SequenceNum] (CodeWidth,   Code) -> CodeVal");
        for (i=0; i<total_code_cnt; i++) {
            printf("\t[%11d] (%9d, %#6x) -> %#7x\n", i, *pWidth++, *pCode++, *pCodeVal++);
        }
        len -= (17+total_code_cnt);
        puts("\t---------------------------------------------------------------------------------------------");
    }
    return 0;
}

int RebuildMCU1X1(struct ABitReader *abr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*dst)[8], int i, int j, bool dump);
int RebuildMCU1X2(struct ABitReader *abr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*dst)[8], int i, int j, bool dump);
int RebuildMCU2X1(struct ABitReader *abr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*dst)[8], int i, int j, bool dump);
int RebuildMCU2X2(struct ABitReader *abr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*dst)[8], int i, int j, bool dump);

//map comp_id to quantized_table
int parseSOF(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), SOF offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    param->bit_width = abr->getBits(8);
    param->height = abr->getBits(16);
    param->width = abr->getBits(16);
    printf("\tpic resolution => (%d x %d)\n", param->width, param->height);
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
    if (param->hfactor[0]==1 && param->vfactor[0]==1) {
        param->RebuildMCUFunc = RebuildMCU1X1;
    } else if (param->hfactor[0]==1 && param->vfactor[0]==2) {
        param->RebuildMCUFunc = RebuildMCU1X2;
    } else if (param->hfactor[0]==2 && param->vfactor[0]==1) {
        param->RebuildMCUFunc = RebuildMCU2X1;
    } else if (param->hfactor[0]==2 && param->vfactor[0]==2) {
        param->RebuildMCUFunc = RebuildMCU2X2;
    } else {
        puts("error sampling factor!");
        assert(0);
    }

    return 0;
}


int parseDRI(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), DRI offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    param->restart_interval = abr->getBits(16);

    return 0;
}

//map comp_id to huffman_table
int parseSOS(ABitReader* abr, struct jpegParam* param)
{
    printf("(%s : %d), SOS offset:%#x\n", __func__, __LINE__, abr->getOffset());
    int len = abr->getBits(16);
    int comp_cnt = abr->getBits(8);
    int i;
    for (i=0; i<comp_cnt; i++) {
        param->ht_comp_id[i] = abr->getBits(8);
        CHECK_EQ(param->ht_comp_id[i], i+1);
        int ht_idx = abr->getBits(8);
        param->ht_idx[i] = ht_idx;
        int dc_idx = ht_idx>>4;
        int ac_idx = ht_idx & 0x0f;
        printf("\tcolor_id[%d] use DC_table[%d], AC_table[%d]\n", param->ht_comp_id[i], dc_idx, ac_idx);
    }
    uint32_t baseline_flag = abr->getBits(24);
    puts("\tonly support baseline profile! should pass for most cases!");
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
            case DRI:
                parseDRI(&abr, param);
                break;
            case SOS:
                parseSOS(&abr, param);
                exit_flag = true;
                break;
            case EOI:
                break;
            default:
                printf("unknown tag[%#x] at position[%#x]\n", jpeg_tag, abr.getOffset());
                abr.skipBits((abr.getBits(16)-2)*8);
                //CHECK(0);
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
// this primary version is not hight efficient and can not skip 0x00 that follows 0xff
int HuffmanDecode(struct ABitReader *abr, struct jpegParam *param, int color, int idx, int *freq, int *coef)
{
    uint8_t *pCodeCnt = param->pHTCodeCnt[idx][color];
    uint16_t *pCode = param->pHTCode[idx][color];
    uint8_t *pCodeVal = param->pHTCodeVal[idx][color];

    //printf("\tHuffmanDecodeDebug => offset:%#x, val:%#x, bitsLeft:%d\n", abr->getOffset(), *abr->data(), abr->numBitsLeftInPart());
    int i = 1;
    uint16_t code_high = 0;
    uint16_t code_low = 0;
    uint16_t code = 0;
    int cur_width = 0;
    while (i < 16) {
        if (pCodeCnt[i] != 0) {
            int new_width = (i+1) - cur_width;
            code_high = code_high << new_width;
            if (*abr->data() == 0xff) {
                //printf("oops! offset:%x, left_bits:%d\n", abr->getOffset(), left_bits);
                int cnt = new_width;
                do {
                    code_low = code_low<<1;
                    code_low += abr->getBits(1);
                    if (*abr->data() == 0) {
                        abr->skipBits(8);
                        printf("encounter 0x00 after 0xff, skip 0x00!\n");
                    }
                } while (--cnt);
            } else {
                code_low = abr->getBits(new_width);
                //printf("i:%d, code_low:%#x\n", i, code_low);
            }
            cur_width = i+1;
            code = code_high + code_low;
            code_high = code;
            //printf("cur_width:%d, CodeCnt:%d, code:%#x, left_bits:%d\n", cur_width, pCodeCnt[i], code, abr->numBitsLeftInPart());
            bool find = true;
            int cnt = 0;
            while (*pCode++ != code) {
                pCodeVal++;
                if (++cnt == pCodeCnt[i]) {
                    find = false;
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
                    }
                    return 0;
                } else if (idx == 1){ //AC
                    *freq = *pCodeVal >> 4;
                    if (*pCodeVal == 0xf0) {
                        *freq = 16;
                        *coef = 0;
                        return 0;
                    } else if (*pCodeVal == 0x00) { // EOB: all of below coefficient is 0
                        *freq = 0; //none sense
                        *coef = 0;
                        return EOB;  //careful for this ret, it means EOB!
                    } else {
                        int coef_bw = *pCodeVal & 0x0f;
                        if (code_val >> (coef_bw-1)) {    //  MSB(Most Significant Bit) = 1
                            *coef = code_val;
                        } else {                // MSB = 0
                            uint32_t tmp1 = (0xffffffff>>coef_bw)<<coef_bw | code_val;
                            uint32_t tmp2 = ~tmp1;
                            *coef = -tmp2;
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

    printf("fatal error! huffman decode failed! i=%d\n", i);
    TRESPASS();

    return 0;
}

//implement huffman decode again!
//(freq, coef): freq 0 before coef for RLC
int HuffmanDecode2(struct ABitReader *abr, struct jpegParam *param, COLOR_TYPE color, int idx, int *freq, int *coef)
{
    uint8_t *pCodeCnt = param->pHTCodeCnt[idx][color];
    uint16_t *pCode = param->pHTCode[idx][color];
    uint8_t *pCodeVal = param->pHTCodeVal[idx][color];

    //printf("\tHuffmanDecodeDebug => offset:%#x, val:%#x, bitsLeft:%d\n", abr->getOffset(), *abr->data(), abr->numBitsLeftInPart());

    /** 0x00 follow 0xff by code standard, we need skip 0x00
     *  we may encounter 0xff not in the start of one scan, so need process this case
     */
    bool pre_skip_flag = false;
    int distance = -1;
    // how many bits from current position when come to 0x00 that follows 0xff?
    // we detect 4 Bytes for one scan, because it can cover most case except extreme case
    if ((abr->data()[0]==0xff) || (abr->data()[1]==0xff) || (abr->data()[2]==0xff) || (abr->data()[3]==0xff)) {
        pre_skip_flag = true;
        int left_bits = abr->numBitsLeftInPart()%8; //how many bits left in current Byte
        int i = 0;
        for (; abr->data()[i]!=0xff; i++);
        //CHECK_EQ(abr->data()[i+1], 0x00); //may encounter EOI(FF D9)
        distance = i*8 + (left_bits==0?8:left_bits);
        //printf("\tfind 0xff(%#x,%#x,%#x,%#x) left_bits:%d, offset:%#x, distance:%d\n", abr->data()[0],
        //        abr->data()[1], abr->data()[2], abr->data()[3], left_bits, abr->getOffset(), distance);
        //assert(0);
    }

    uint16_t *pCodeStartPos = pCode;
    uint16_t code = 0;
    int cur_code_width = 2; //[2,16], may be 0 in pCodeCnt[cur_code_width-1]
    int last_code_width = 1;
    code = abr->getBits(1);
    while (cur_code_width <= 16)
    {
          //this part no skip 0x00 process
//        if (pCodeCnt[cur_code_width-1] == 0)
//        {
//            cur_code_width++;
//            continue;
//        }
//        else
//        {
//            int shift_width = cur_code_width-last_code_width;
//            code = (code<<shift_width) + abr->getBits(shift_width);
//            //printf("searching... width[%d/%d], code[%#x]\n", cur_code_width, last_code_width, code);
//            bool find = true;
//            int cnt = 0;
//            while (*pCode++ != code) {
//                if (++cnt == pCodeCnt[cur_code_width-1]) {
//                    find = false;
//                    break;
//                }
//            }

        if (pCodeCnt[cur_code_width-1] == 0)
        {
            if (cur_code_width == distance+1) {
                //printf("[warning] when pCodeCnt[]=0, left_bits_in_Byte[%d], data[%#x] probably be 0x00, skip it!\n", abr->numBitsLeftInPart()%8, abr->data()[0]);
                //printf("[info] pos[%#x], cur_code[%#x], cur_code_widht[%d], distance[%d]!\n", abr->getOffset(), code, cur_code_width, distance);
                int add_bw = abr->numBitsLeftInPart()%8;
                if (add_bw) {
                    code = (code<<add_bw) + abr->getBits(add_bw);
                    CHECK(abr->data()[0]==0);
                    last_code_width += add_bw;
                    //printf("add_bw[%d], cur_code[%#x], code_width[%d/%d]\n", add_bw, code, cur_code_width, last_code_width);
                    //assert(0);
                }
                abr->skipBits(8);
            }
            cur_code_width++;
            continue;
        }
        else
        {
            if (cur_code_width == distance+1) {
                //printf("[warning] when pCodeCnt[]!=0, left_bits_in_Byte[%d], data[%#x] probably be 0x00, skip it!\n", abr->numBitsLeftInPart()%8, abr->data()[0]);
                //printf("[info] pos[%#x], cur_code[%#x], cur_code_widht[%d], distance[%d]!\n", abr->getOffset(), code, cur_code_width, distance);
                int add_bw = abr->numBitsLeftInPart()%8;
                if (add_bw) {
                    code = (code<<add_bw) + abr->getBits(add_bw);
                    CHECK(abr->data()[0]==0);
                    last_code_width += add_bw;
                    //printf("add_bw[%d], cur_code[%#x], code_width[%d/%d]\n", add_bw, code, cur_code_width, last_code_width);
                    //assert(0);
                }
                abr->skipBits(8);
                //assert(0);
            }
            int num = abr->numBitsLeftInPart()%8;
            if (num==0 && abr->data()[0]==0x00 && abr->data()[-1]==0xff) {
                //skip the second pair of 0xff 0x00 when meed continuous two pair
                //printf("Be careful! offset:%#x, [%#x,%#x], [%#x,%#x]\n", abr->getOffset(), abr->data()[-3], abr->data()[-2], abr->data()[-1], abr->data()[0]);
                //assert(0);
                abr->skipBits(8);
            }
            int shift_width = cur_code_width-last_code_width;
            code = (code<<shift_width) + abr->getBits(shift_width);
            //printf("searching... code_width[%d/%d], code[%#x], pos[%#x], bit_left[%d]\n", cur_code_width, last_code_width, code, abr->getOffset(), abr->numBitsLeftInPart());
            bool find = true;
            int cnt = 0;
            while (*pCode++ != code) {
                if (++cnt == pCodeCnt[cur_code_width-1]) {
                    find = false;
                    break;
                }
            }

            if (find == false)
            {
                last_code_width = cur_code_width;
                cur_code_width++;
            }
            else
            {
                int offset = pCode - pCodeStartPos - 1;
                uint8_t code_val = pCodeVal[offset];
                int coef_bitwidth = code_val & 0x0f;
                uint16_t raw_code_val = abr->getBits(coef_bitwidth);
                //printf("color/idx[%d:%d], find code[%#x], coef_bitwidth[%d], code_val[%#x], raw_code_val[%#x]\n", color, idx, code, coef_bitwidth, code_val, raw_code_val);
                if (idx == 0)  //DC
                {
                    *freq = 0;
                    if (raw_code_val >> (coef_bitwidth-1))
                    {
                        *coef = raw_code_val;       //  MSB(Most Significant Bit) = 1, positive value
                    }
                    else
                    {
                        int tmp = ((1<<coef_bitwidth)-1) & (~raw_code_val);
                        *coef = -tmp;
                    }
                    return 0;
                }
                else if (idx == 1) //AC
                {
                    if (code_val == 0xf0)
                    {
                        *freq = 16;
                        *coef = 0;
                        return 0;
                    }
                    else if (code_val == 0x00) // EOB: all of below coefficient is 0
                    {
                        *freq = 0;      //none sense
                        *coef = 0;
                        return EOB;     //careful for this ret, it means EOB!
                    }
                    else
                    {
                        *freq = code_val>>4;
                        if (raw_code_val >> (coef_bitwidth-1))
                        {
                            *coef = raw_code_val;       //  MSB(Most Significant Bit) = 1, positive value
                        }
                        else
                        {
                            int tmp = ((1<<coef_bitwidth)-1) & (~raw_code_val); //reverse every bit when MSB=0
                            *coef = -tmp;
                        }
                        return 0;
                    }
                }
                else
                {
                    TRESPASS();
                    break;
                }
            }
        }
    }

    printf("fatal error! huffman decode failed!\n");
    TRESPASS();

    return 0;
}

//implement huffman decode again!
//(freq, coef): freq 0 before coef for RLC
int HuffmanDecode3(struct ABitReader *abr, struct jpegParam *param, int color, int idx, int *freq, int *coef)
{
    uint8_t *pCodeCnt = param->pHTCodeCnt[idx][color];
    uint16_t *pCode = param->pHTCode[idx][color];
    uint8_t *pCodeVal = param->pHTCodeVal[idx][color];

    //printf("\tHuffmanDecodeDebug => offset:%#x, val:%#x, bitsLeft:%d\n", abr->getOffset(), *abr->data(), abr->numBitsLeftInPart());

    uint16_t *pCodeStartPos = pCode;
    uint16_t code = 0;
    int cur_code_width = 2; //[2,16], may be 0 in pCodeCnt[cur_code_width-1]
    code = abr->getBits(1);
    while (cur_code_width <= 16)
    {
        if (pCodeCnt[cur_code_width-1] == 0)
        {
            int num = abr->numBitsLeftInPart()%8;
            if (num==0 && abr->data()[0]==0x00 && abr->data()[-1]==0xff) {
                //printf("Be careful! offset:%#x, [%#x,%#x], [%#x,%#x]\n", abr->getOffset(), abr->data()[-3], abr->data()[-2], abr->data()[-1], abr->data()[0]);
                abr->skipBits(8);
            }
            code = (code<<1) + abr->getBits(1);
            cur_code_width++;
            continue;
        }
        else
        {
            int num = abr->numBitsLeftInPart()%8;
            if (num==0 && abr->data()[0]==0x00 && abr->data()[-1]==0xff) {
                //printf("Be careful! offset:%#x, [%#x,%#x], [%#x,%#x]\n", abr->getOffset(), abr->data()[-3], abr->data()[-2], abr->data()[-1], abr->data()[0]);
                abr->skipBits(8);
            }
            code = (code<<1) + abr->getBits(1);
            //printf("searching... code_width[%d], code[%#x], pos[%#x], bit_left[%d]\n", cur_code_width, code, abr->getOffset(), abr->numBitsLeftInPart());
            bool find = true;
            int cnt = 0;
            while (*pCode++ != code) {
                if (++cnt == pCodeCnt[cur_code_width-1]) {
                    find = false;
                    break;
                }
            }

            if (find == false)
            {
                cur_code_width++;
            }
            else
            {
                int offset = pCode - pCodeStartPos - 1;
                uint8_t code_val = pCodeVal[offset];
                int coef_bitwidth = code_val & 0x0f;
                uint16_t raw_code_val = abr->getBits(coef_bitwidth);
                //printf("color/idx[%d:%d], find code[%#x], coef_bitwidth[%d], code_val[%#x], raw_code_val[%#x]\n", color, idx, code, coef_bitwidth, code_val, raw_code_val);
                if (idx == 0)  //DC
                {
                    *freq = 0;
                    if (raw_code_val >> (coef_bitwidth-1))
                    {
                        *coef = raw_code_val;       //  MSB(Most Significant Bit) = 1, positive value
                    }
                    else
                    {
                        int tmp = ((1<<coef_bitwidth)-1) & (~raw_code_val);
                        *coef = -tmp;
                    }
                    return 0;
                }
                else if (idx == 1) //AC
                {
                    if (code_val == 0xf0)
                    {
                        *freq = 16;
                        *coef = 0;
                        return 0;
                    }
                    else if (code_val == 0x00) // EOB: all of below coefficient is 0
                    {
                        *freq = 0;      //none sense
                        *coef = 0;
                        return EOB;     //careful for this ret, it means EOB!
                    }
                    else
                    {
                        *freq = code_val>>4;
                        if (raw_code_val >> (coef_bitwidth-1))
                        {
                            *coef = raw_code_val;       //  MSB(Most Significant Bit) = 1, positive value
                        }
                        else
                        {
                            int tmp = ((1<<coef_bitwidth)-1) & (~raw_code_val); //reverse every bit when MSB=0
                            *coef = -tmp;
                        }
                        return 0;
                    }
                }
                else
                {
                    TRESPASS();
                    break;
                }
            }
        }
    }

    printf("\tfatal error! huffman decode failed!\n");
    printf("\tdump info => offset:%#x, val:%#x, bitsLeft:%d\n", abr->getOffset(), *abr->data(), abr->numBitsLeftInPart());
    TRESPASS();

    return 0;
}

// IDPCM + IRLC
//color: 0-Y; 1-CbCr
//block_size: 8x8
int RebuildBlock(struct ABitReader *abr, struct jpegParam *param, COLOR_TYPE color, int *block, bool dump)
{
    int *ptr = block;
    memset(ptr, 0, 8*8*4);

    int cnt, coef;

    //DC
    int ret = HuffmanDecode3(abr, param, color, COEF_DC, &cnt, &coef);
    switch (param->cur_type) {
        case 0:
            coef += param->YDC_last;
            param->YDC_last = coef;
            break;
        case 1:
            coef += param->CbDC_last;
            param->CbDC_last = coef;
            break;
        case 2:
            coef += param->CrDC_last;
            param->CrDC_last = coef;
            break;
        default:
            TRESPASS();
            break;
    }
    *ptr++ = coef;

    //AC
    int i;
    for (i=1; i<64; i++) {
        ret = HuffmanDecode3(abr, param, color, COEF_AC, &cnt, &coef);
        while(cnt--) {
            *ptr++ = 0;
        }
        *ptr++ = coef;

        if (ret == EOB) {
            break;
        }
    }
    if (i==64) {
        puts("Oops! something goes wrong!");
        TRESPASS();
    }

    //dump block
    if (dump) {
        printf("----after huffman decode, color[%d]----\n", color);
        for (i=0; i<8; i++) {
            for (int j=0; j<8; j++) {
                printf("%4d  ", *block++);
            }
            puts("");
        }
        puts("");
    }

    return 0;
}

// IQS
int JpegDequantization(struct ABitReader *abr, struct jpegParam *param, int *ptr_block, bool dump)
{
    int color_id = param->cur_type==0 ? 0:1;
    uint8_t *pQT = param->pQT[color_id];
    for (int i=0; i<64; i++) {
        ptr_block[i] *= pQT[i];
    }

    if (dump) {
        puts("----after dequantization----");
        for (int i=0; i<64; i++) {
            printf("%4d  ", ptr_block[i]);
            if (i % 8 == 7)
                puts("");
        }
        puts("");
    }

    return 0;
}

// IZigZag
int JpegReZigZag(struct ABitReader *abr, struct jpegParam *param, int *dst_block, const int *src_block, bool dump)
{
    for (int i=0; i<64; i++) {
        dst_block[i] = src_block[zigzag[i]];
    }

    if (dump) {
        puts("----after rezigzag----");
        for (int i=0; i<64; i++) {
            printf("%4d  ", dst_block[i]);
            if (i % 8 == 7)
                puts("");
        }
        puts("");
    }

    return 0;
}

int IDCT2(float (*dst)[8], int (*block)[8], bool dump)
{
    float trans_matrix[8][8] = {
        {0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,    0.3536,},
        {0.4904,    0.4157,    0.2778,    0.0975,   -0.0975,   -0.2778,   -0.4157,   -0.4904,},
        {0.4619,    0.1913,   -0.1913,   -0.4619,   -0.4619,   -0.1913,    0.1913,    0.4619,},
        {0.4157,   -0.0975,   -0.4904,   -0.2778,    0.2778,    0.4904,    0.0975,   -0.4157,},
        {0.3536,   -0.3536,   -0.3536,    0.3536,    0.3536,   -0.3536,   -0.3536,    0.3536,},
        {0.2778,   -0.4904,    0.0975,    0.4157,   -0.4157,   -0.0975,    0.4904,   -0.2778,},
        {0.1913,   -0.4619,    0.4619,   -0.1913,   -0.1913,    0.4619,   -0.4619,    0.1913,},
        {0.0975,   -0.2778,    0.4157,   -0.4904,    0.4904,   -0.4157,    0.2778,   -0.0975,},
    };

    float tmp[8][8];

    float t=0;
    int i,j,k;
    for(i=0;i<8;i++)  //same as A'*I
	{
        for(j=0;j<8;j++)
		{
            t = 0;
            for(k=0; k<8; k++)
			{
                t += trans_matrix[k][i] * block[k][j]; //trans_matrix's ith column * block's jth column
			}
            tmp[i][j] = t;
        }
    }

    for(i=0; i<8; i++)  //same as tmp*A
	{
        for(j=0; j<8; j++)
		{
            t=0;
            for(k=0; k<8; k++)
			{
                t += tmp[i][k] * trans_matrix[k][j];
			}
            dst[i][j] = t;
        }
    }

    if (dump) {
        puts("----after idct2----");
        for (i=0; i<8; i++) {
            for (j=0; j<8; j++) {
                printf("%3.4f  ", dst[i][j]);
            }
            puts("");
        }
        puts("");
    }

    return 0;
}

// ILevelOffset
int JpegReLevelOffset(float (*block)[8], bool dump)
{
    int i, j;

    for (i=0; i<8; i++) {
        for (j=0; j<8; j++) {
            block[i][j] += 128;
        }
    }

    if (dump) {
        puts("----after ileveloffset----");
        for (i=0; i<8; i++) {
            for (j=0; j<8; j++) {
                printf("%3.4f  ", block[i][j]);
            }
            puts("");
        }
        puts("");
    }

    return 0;
}

inline int JpegCopyYUV(uint8_t (*dst)[8], float (*src)[8], bool dump)
{
    int i,j;
    for (i=0; i<8; i++) {
        for (j=0; j<8; j++) {
            dst[i][j] = (uint8_t)src[i][j];
        }
    }

    return 0;
}

// mcu_size:8x8, Y, U, V
int RebuildMCU1X1(struct ABitReader *pabr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*pdst)[8], int i, int j, bool dump)
{
    uint8_t yuv[8][8];
    float dst[8][8];

    //puts("--------Y--------");
    param->cur_type = 0;
    RebuildBlock(pabr, param, COLOR_Y, &block1[0][0], dump);          //IDPCM + IRLC
    JpegDequantization(pabr, param, &block1[0][0], dump);             //IQS
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);    //IZigZag
    IDCT2(&dst[0], &block2[0], dump);                                 //IDCT
    JpegReLevelOffset(&dst[0], dump);                                 //ILevelOffset
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++)
        memcpy(param->py_data+i*8*param->width+j*8+k*param->width, yuv[k], 8);

    //puts("--------Cb--------");
    param->cur_type = 1;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++)
        memcpy(param->pu_data+i*8*param->width+j*8+k*param->width, yuv[k], 8);

    //puts("--------Cr--------");
    param->cur_type = 2;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++)
        memcpy(param->pv_data+i*8*param->width+j*8+k*param->width, yuv[k], 8);

    return 0;
}

// mcu_size:16x8, Y00+Y10, U, V
int RebuildMCU1X2(struct ABitReader *pabr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*pdst)[8], int i, int j, bool dump)
{
    uint8_t yuv[8][8];
    float dst[8][8];

    for (int cnt=0; cnt<2; cnt++) {
        //printf("--------Y[%d]--------\n", cnt);
        param->cur_type = 0;
        RebuildBlock(pabr, param, COLOR_Y, &block1[0][0], dump);          //IDPCM + IRLC
        JpegDequantization(pabr, param, &block1[0][0], dump);             //IQS
        JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);    //IZigZag
        IDCT2(&dst[0], &block2[0], dump);                                 //IDCT
        JpegReLevelOffset(&dst[0], dump);                                 //ILevelOffset
        JpegCopyYUV(&yuv[0], &dst[0], dump);
        for (int k=0; k<8; k++) {
            if (cnt==0)
                memcpy(param->py_data+i*16*1024+j*8+k*1024, yuv[k], 8);
            else if (cnt==1)
                memcpy(param->py_data+i*16*1024+8*1024+j*8+k*1024, yuv[k], 8);
        }
    }

    //puts("--------Cb--------");
    param->cur_type = 1;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++) {
        memcpy(param->pu_data+i*8*1024+j*8+k*1024, yuv[k], 8);
    }

    //puts("--------Cr--------");
    param->cur_type = 2;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++) {
        memcpy(param->pv_data+i*8*1024+j*8+k*1024, yuv[k], 8);
    }

    return 0;
}

// mcu_size:8x16, Y00+Y01, U, V
int RebuildMCU2X1(struct ABitReader *pabr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*pdst)[8], int i, int j, bool dump)
{
    uint8_t yuv[8][8];
    float dst[8][8];
    for (int cnt=0; cnt<2; cnt++) {
        //printf("--------Y[%d]--------\n", cnt);
        param->cur_type = 0;
        RebuildBlock(pabr, param, COLOR_Y, &block1[0][0], dump);          //IDPCM + IRLC
        JpegDequantization(pabr, param, &block1[0][0], dump);             //IQS
        JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);    //IZigZag
        IDCT2(&dst[0], &block2[0], dump);                                 //IDCT
        JpegReLevelOffset(&dst[0], dump);                                 //ILevelOffset
        JpegCopyYUV(&yuv[0], &dst[0], dump);
        for (int k=0; k<8; k++) {
            if (cnt==0)
                memcpy(param->py_data+i*8*1024+j*16+k*1024, yuv[k], 8);
            else if (cnt==1)
                memcpy(param->py_data+i*8*1024+8+j*16+k*1024, yuv[k], 8);
        }
    }

    //puts("--------Cb--------");
    param->cur_type = 1;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++)
        memcpy(param->pu_data+i*8*512+j*8+k*512, yuv[k], 8);

    //puts("--------Cr--------");
    param->cur_type = 2;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++)
        memcpy(param->pv_data+i*8*512+j*8+k*512, yuv[k], 8);

    return 0;
}

// mcu_size:16x16, Y00+Y01+Y10+Y11, U, V
int RebuildMCU2X2(struct ABitReader *pabr, struct jpegParam *param, int (*block1)[8], int (*block2)[8], float (*pdst)[8], int i, int j, bool dump)
{
    uint8_t yuv[8][8];
    float dst[8][8];

    for (int cnt=0; cnt<4; cnt++) {
        //printf("--------Y[%d]--------\n", cnt);
        param->cur_type = 0;
        RebuildBlock(pabr, param, COLOR_Y, &block1[0][0], dump);          //IDPCM + IRLC
        JpegDequantization(pabr, param, &block1[0][0], dump);             //IQS
        JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);    //IZigZag
        IDCT2(&dst[0], &block2[0], dump);                                 //IDCT
        JpegReLevelOffset(&dst[0], dump);                                 //ILevelOffset
        JpegCopyYUV(&yuv[0], &dst[0], dump);
        for (int k=0; k<8; k++) {
            if (cnt==0)
                memcpy(param->py_data+i*16*1024+j*16+k*1024, yuv[k], 8);
            else if (cnt==1)
                memcpy(param->py_data+i*16*1024+j*16+8+k*1024, yuv[k], 8);
            else if (cnt==2)
                memcpy(param->py_data+i*16*1024+8*1024+j*16+k*1024, yuv[k], 8);
            else if (cnt==3)
                memcpy(param->py_data+i*16*1024+8*1024+8+j*16+k*1024, yuv[k], 8);
        }
    }

    //puts("--------Cb--------");
    param->cur_type = 1;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++) {
        memcpy(param->pu_data+i*8*512+j*8+k*512, yuv[k], 8);
    }

    //puts("--------Cr--------");
    param->cur_type = 2;
    RebuildBlock(pabr, param, COLOR_UV, &block1[0][0], dump);
    JpegDequantization(pabr, param, &block1[0][0], dump);
    JpegReZigZag(pabr, param, &block2[0][0], &block1[0][0], dump);
    IDCT2(&dst[0], &block2[0], dump);
    JpegReLevelOffset(&dst[0], dump);
    JpegCopyYUV(&yuv[0], &dst[0], dump);
    for (int k=0; k<8; k++) {
        memcpy(param->pv_data+i*8*512+j*8+k*512, yuv[k], 8);
    }

    return 0;
}

// IDPCM + IRLC + IQS + IZigzag + IDCT + ILevelOffset
int JpegDecode(FileSource *fs, struct jpegParam *param, int offset)
{
    int width, height;
    int block1[8][8];
    int block2[8][8];
    float dst[8][8];
    uint8_t yuv[8][8];
    memset(&block1, 0, 64*4);

    off64_t file_sz;
    fs->getSize(&file_sz);
    uint8_t *buf = (uint8_t*)malloc(file_sz);
    fs->readAt(offset, buf, file_sz);
    struct ABitReader abr(buf, file_sz);

    struct timeval begin_tv, end_tv;
    gettimeofday(&begin_tv, NULL);

    param->py_data = (uint8_t*)malloc(param->width*param->height);
    param->pu_data = (uint8_t*)malloc(param->width*param->height);
    param->pv_data = (uint8_t*)malloc(param->width*param->height);

    printf("entropy begin! offset[%#x]\n", offset);
    int i,j;
    int *pos1 = &block1[0][0];
    int *pos2 = &block2[0][0];

    bool dump = false;
    int mcu_row = param->height/8/param->vfactor[0];
    int mcu_col = param->width/8/param->hfactor[0];
    for (i=0; i<mcu_row; i++) {
        for (j=0; j<mcu_col; j++) {
            //printf("------------------block(%d,%d)----------------------\n", i, j);
            if (param->restart_interval) {
                if (param->scan_mcu_cnt++ == param->restart_interval) {
                    //printf("need restart scan: [%#x] [%#x] [%#x] [%#x]\n", abr.data()[0], abr.data()[1], abr.data()[2], abr.data()[3]);
                    int left_bits = abr.numBitsLeftInPart()%8;
                    abr.skipBits(left_bits+16);
                    param->scan_mcu_cnt = 1;
                    param->YDC_last = 0;
                    param->CbDC_last = 0;
                    param->CrDC_last = 0;
                }
            }
            if (i==56 && j==63) {
                puts("-------break--------");
                goto SKIP_SCAN;
            }
            param->RebuildMCUFunc(&abr, param, &block1[0], &block2[0], &dst[0], i, j, dump);
            //assert(0);
        }
    }
    printf("entropy end! offset[%#x], cur_data[%#x], last_two_bytes:[%#x %#x] should be EOI:[0xFF 0xD9]\n",
            abr.getOffset(), abr.data()[0], abr.data()[1], abr.data()[2]);

SKIP_SCAN:
    gettimeofday(&end_tv, NULL);
    long time_dura_us = (end_tv.tv_sec - begin_tv.tv_sec)*1000000 + (end_tv.tv_usec - begin_tv.tv_usec);
    printf("jpeg entropy time duration: [%ld] us\n", time_dura_us);

    FILE *yuv_fp = fopen("test_pic.yuv", "wb");

    // yuv444 for 1x1.jpg
    //fwrite(param->py_data, 1, param->width*param->height, yuv_fp);
    //fwrite(param->pu_data, 1, param->width*param->height, yuv_fp);
    //fwrite(param->pv_data, 1, param->width*param->height, yuv_fp);
    fwrite(param->py_data, 1, param->width*57*8, yuv_fp);
    fwrite(param->pu_data, 1, param->width*57*8, yuv_fp);
    fwrite(param->pv_data, 1, param->width*57*8, yuv_fp);

    // nv21 for 1x1.jpg
    //fwrite(y_data, 1, 1024*1024, yuv_fp);
    //for (int i=0; i<1024; i+=2)
    //{
    //    for (int j=0; j<1024; j+=2)
    //    {
    //        fwrite(v_data+i*1024+j, 1, 1, yuv_fp);
    //        fwrite(u_data+i*1024+j, 1, 1, yuv_fp);
    //    }
    //}

    // yuv420_3_plane for 2x2.jpg
    //fwrite(param->py_data, 1, 1024*1024, yuv_fp);
    //fwrite(param->pu_data, 1, 1024*1024/4, yuv_fp);
    //fwrite(param->pv_data, 1, 1024*1024/4, yuv_fp);

    // nv21 for 1x2.jpg
    //fwrite(param->py_data, 1, 1024*1024, yuv_fp);
    //for (int i=0; i<512; i++)
    //{
    //    for (int j=0; j<1024; j+=2)
    //    {
    //        fwrite(param->pv_data+i*1024+j, 1, 1, yuv_fp);
    //        fwrite(param->pu_data+i*1024+j, 1, 1, yuv_fp);
    //    }
    //}

    // yuv420_3_plane for 2x1.jpg
    //fwrite(param->py_data, 1, 1024*1024, yuv_fp);
    //for (int i=0; i<1024; i+=2)
    //    for (int j=0; j<64; j++)
    //        fwrite(param->pu_data+i*512+j*8, 1, 8, yuv_fp);
    //for (int i=0; i<1024; i+=2)
    //    for (int j=0; j<64; j++)
    //        fwrite(param->pv_data+i*512+j*8, 1, 8, yuv_fp);

    fclose(yuv_fp);

    free(buf);

    return 0;
}

int main(void)
{
    cout<<"------begin-------"<<endl;
    FileSource *pFS = new FileSource("graphis-1x1.jpg");
    struct jpegParam param;
    memset(&param, 0, sizeof(param));

    off64_t sz;
    pFS->getSize(&sz);
    cout<<"sz: "<<sz<<endl;

    uint8_t *buf = (uint8_t*)malloc(4096);
    pFS->readAt(0, buf, 4096);
    struct ABitReader abr(buf, 4096);

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
