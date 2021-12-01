/*
 *  2020.04.20 created by Kelly @nepes
 *  module regarding Image
 *  Image.hpp
 *  Copyright (c) 2020, nepes inc, All rights reserved.
 *
 */

#pragma once
#include <stdio.h>
#include <string>
#include "os_api.hpp"
#include "bitmap.hpp"
#include "lodepng.hpp"



using namespace std;

#define WH 255

#if 0 //for reference
enum channel_mode {
    rgb_mode   = 0,
    bgr_mode   = 1,
    rgba_mode  = 2,
    bgra_mode  = 3,
    gray1_mode = 4, // gray 1 channel
    gray3_mode = 5, // gray 3 channels
};

enum color_plane {
    blue_plane  = 0,
    green_plane = 1,
    red_plane   = 2
};
#endif

typedef struct _imgdata
{
    int id;
    int width;
    int height;
    int area;
    int channel;
    int channel_mode;
    int hh;
    int mm;
    int ss;
    unsigned char* buf;
    int remain_cnt;
}IMG;

IMG *CreateImage(int width, int height, int ch = 0);
IMG* CreateImage(int width, int height, int ch, unsigned char* data);
int CopyImage(IMG* in, IMG* out);
void DestroyImage(IMG* target);

void ClearImage(IMG* in);
void CropImage(IMG* in, IMG* out, int sx, int sy);
void ChangeCanvas(IMG* in, int new_w, int new_h);
void ConvertToGray(IMG* in, IMG* out);
void ConvertChannel(IMG* img, int channel);
void ResizeImage(IMG* img, int width, int height,int use_cubic);
void LoadImage(IMG* img, char* filename);
void SaveImage(IMG* img, char* filename);
void SaveImagePNG(IMG* img, char* filename);

void ConvertToAlpha(IMG* in, IMG* out);
void ConvertToGray1(IMG* in, IMG* out);
void ConvertToGray3(IMG* in, IMG* out);

IMG *BMP_to_IMG(bitmap *bmp);
IMG *BMP_to_IMG(bitmap *bmp, int fno, int curss);

bitmap *IMG_to_BMP(IMG *img);
void CheckImageSpec(IMG* img);
int CheckTheImageGrayscale(IMG* img);
