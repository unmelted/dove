/*
 *  2020.04.20 created by Kelly @nepes
 *  module regarding Image
 *  Image.cpp
 *  Copyright (c) 2020, nepes inc, All rights reserved.
 *
 */
#include <cassert>
#include "Image.hpp"
#include "TimeUtil.hpp"

static int img_cnt = 0;
static bool fcnt = false;

int GetCH(int ch_mode)
{
    int ch = 0;
    switch(ch_mode){
    case gray1_mode:
        ch = 1;
        break;
    case rgb_mode:
    case bgr_mode:
    case gray3_mode:
        ch = 3;
        break;
    case rgba_mode:
    case bgra_mode:
        ch = 4;
       break;
    }

    return ch;
}

IMG* CreateImage(int width, int height, int ch_mode)
{
    IMG* target = (IMG*)g_os_malloc(sizeof(IMG));
    printf("create iamge start empty");    
    int ch = GetCH(ch_mode);
    target->width = width;
    target->height = height;
    target->area = width * height;
    target->buf = (unsigned char *)g_os_malloc(sizeof(unsigned char)* width * height * ch);
    target->channel_mode = ch_mode;
    target->channel = ch;
    target->remain_cnt = 1;
    target->hh = 0;
    target->mm = 0;
    target->ss = 0;
    memset(target->buf, WH, sizeof(unsigned char)* width * height* ch);

    img_cnt++;
//    printf("cimg0[%d] %p ", img_cnt, target); fflush(stdout);

    if(img_cnt == 20 && fcnt == false)
        fcnt = true;
    return target;
}

IMG* CreateImage(int width, int height, int ch_mode, unsigned char* data)
{
    printf("create iamge start ");    
    IMG* target = (IMG*)g_os_malloc(sizeof(IMG));
    int ch = GetCH(ch_mode);
    target->width = width;
    target->height = height;
    target->area = width * height;
    target->buf = (unsigned char *)g_os_malloc(sizeof(unsigned char)* width * height * ch);
    target->channel_mode = ch_mode;
    target->channel = ch;
    target->remain_cnt = 1;
    target->hh = 0;
    target->mm = 0;
    target->ss = 0;

    memcpy(target->buf, data, sizeof(unsigned char)* width * height * ch);
    img_cnt++;
//    printf("cimg1[%d] %p ", img_cnt, target); fflush(stdout);
    if(img_cnt == 20 && fcnt == false)
        fcnt = true;

    return target;
}

void DestroyImage(IMG* target)
{
    if(target == NULL)
    {
        Logger("destory image assert 0 \n"); fflush(stdout);
        assert(0);
    }


    static int tcnt;
    tcnt++;
    if(tcnt % 100 == 0)
    {
        tcnt = 0;
        Logger("DSTIMG %p %d\n",target,target->remain_cnt);
    }

    target->remain_cnt--;

    if(target->remain_cnt < 0)
    {
        Logger("destory image remain_cnt < 0 %p \n", target); fflush(stdout);
        assert(0);
    }

    if(target->remain_cnt > 0)
    {
//        printf("dstimg remncnt : %d,  %p ", target->remain_cnt, target);
        return;
    }


    if(target->buf != NULL)
        g_os_free(target->buf);

    //printf(" . "); fflush(stdout);
    g_os_free(target);
    img_cnt--;
    //printf(". cnt[%d] \n", img_cnt); fflush(stdout);

    if(fcnt == true && (img_cnt < 0))
    {
        Logger("abnormal que size img_cnt %d\n", img_cnt); fflush(stdout);
//        assert(0);
    }
}

int CopyImage(IMG* src, IMG* dst)
{
    if(src->width != dst->width ||
       src->height != dst->height ||
       src->channel != dst->channel )
    {
        Logger("cp image err 1 \n");
        return -1;
    }

    if(dst->buf == NULL || src->buf == NULL)
    {
        Logger("cp image err 2 \n");
        return -2;
    }

    memcpy(dst->buf, src->buf, sizeof(unsigned char)*src->area * src->channel);
    dst->channel_mode = src->channel_mode;
    dst->channel = src->channel;
    dst->id = src->id;
    dst->hh = src->hh;
    dst->mm = src->mm;
    dst->ss = src->ss;

    return 0;
}

void ClearImage(IMG* in)
{
    memset(in->buf, 0, sizeof(unsigned char)*in->area * in->channel);
}

void CropImage(IMG* in, IMG* out, int sx, int sy)
{
    if(sy + out->height > in->height)
    {
        Logger("invalid parameter for crop : in height %d, sy %d, out height = %d\n",in->height, sy, out->height);
        assert(0);
    }
    if(sx + out->width > in->width)
    {
        Logger("invalid parameter for crop : in width %d, sx %d, out width = %d\n",in->width, sx, out->width);
        assert(0);
    }
    if(in->channel != out-> channel)
    {
        Logger("invalid parameter for crop : in channel %d, out channel %d\n",in->channel, out->channel);
        assert(0);
    }
    int in_line = in->width * in->channel;
    int out_line = out->width * out->channel;
    int cpy_size = sizeof(unsigned char) * out_line;
    unsigned char *src = &in->buf[ (sy * in->channel * in->width ) + (sx * in->channel) ];
    unsigned char *dst = out->buf;

    for(int y = 0 ; y < out->height ; y++)
    {
        memcpy(dst,src,cpy_size);
        src += in_line;
        dst += out_line;

    }

    out->hh = in->hh;
    out->mm = in->mm;
    out->ss = in->ss;

}

void CheckImageSpec(IMG* img)
{
    printf(":::: IMG :::: \n");
    printf("Channel mode : %d \n", img->channel_mode);
    printf("Channel : %d \n", img->channel);
    printf("Width %d, Height %d \n", img->width, img->height);
    printf(":::: IMG :::: \n");
}

void ChangeCanvas(IMG* in, int new_w, int new_h)
{
    if(in->buf != NULL)
        g_os_free(in->buf);

    in->width = new_w;
    in->height = new_h;
    in->area = new_w * new_h;

    in->buf = (unsigned char*)g_os_malloc(sizeof(unsigned char)* in->area * in->channel);
}

void ConvertToAlpha(IMG* in, IMG* out)
{
    int width, height;
    int idx1=0,idx2=0;
    width = in->width;
    height = in->height;
    for(int i = 0 ; i < in->area; i++)
    {
        out->buf[idx2 + 0] = in->buf[idx1 + 0];
        out->buf[idx2 + 1] = in->buf[idx1 + 1];
        out->buf[idx2 + 2] = in->buf[idx1 + 2];
        out->buf[idx2 + 3] = 0xFF;
        idx1 += 3;
        idx2 += 4;
    }
}

void ConvertToGray1(IMG* in, IMG* out)
{
    int ri, gi, bi = 0;

    double r, g, b;
    int ig;
    unsigned char gray;

    if(in->channel_mode == bgr_mode)
    {
        bi = 0, gi = 1, ri = 2;
    }
    else
    {
        bi = 2, gi = 1, ri = 0;
    }

    int l1 = in->width * 3;
    for(int i = 0; i < in->height; i ++)
    {
        for(int j = 0 ; j < in->width; j ++)
        {
            r = in->buf[i*l1 + j * 3 + ri] * 0.299;;
            g = in->buf[i*l1 + j * 3 + gi] * 0.587;
            b = in->buf[i*l1 + j * 3 + bi] * 0.144;

            ig = floor((r + g + b + 0.5));
            if(ig > 255) ig = 255;
            gray = ig;

            out->buf[i*in->width + j] = gray;

            //if(i<16)out->buf[i*in->width + j] = 255;
        }
    }

    out->hh = in->hh;
    out->mm = in->mm;
    out->ss = in->ss;

    out->channel_mode = 0;
}

void ConvertToGray3(IMG* in, IMG* out)
{
    int rr, gg, bb = 0;
    double r, g, b;
    unsigned char gray;

    for(int i = 0; i < in->height; i ++)
    {
        for(int j = 0 ; j < in->width; j ++)
        {

            rr = in->buf[i*in->width + j * in->channel];
            gg = in->buf[i*in->width + j * in->channel + 1];
            gg = in->buf[i*in->width + j * in->channel + 2];

            r= rr * 0.299;
            g= gg * 0.587;
            b= bb * 0.144;

            gray = floor((r + g + b + 0.5));
            out->buf[i*in->width + j] = gray;
        }
    }

    out->channel_mode = 0;
}

void Convert13(IMG * img)
{
    for(unsigned char* itr = img->buf ; itr < img->buf + (img->channel * img->area) ; itr += img->channel)
    {
        std::swap(*(itr+0), *(itr+2));
    }
}

void ConvertChannel(IMG *img, int channel_mode)
{
    if(channel_mode == img->channel_mode) return;

    int mode1 = img->channel_mode;
    int mode2 = channel_mode;

    if( (mode1 == bgr_mode && mode2 == rgb_mode) || (mode1 == rgb_mode && mode2 == bgr_mode) )
    {
        Convert13(img);
    }
    else if( (mode1 == bgr_mode || mode1 == rgb_mode) && mode2 == gray1_mode )
    {
        //ConvertToGray1(img);
        assert(0);
    }
    else if( (mode1 == bgr_mode || mode1 == rgb_mode) && mode2 == gray3_mode )
    {
        //ConvertToGray3(img);
        assert(0);
    }


    img->channel_mode = channel_mode;
}

double cubicInterpolate (double p[4], double x) {
	return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
}

double bicubicInterpolate (double p[4][4], double x, double y) {
	double arr[4];
	arr[0] = cubicInterpolate(p[0], x);
	arr[1] = cubicInterpolate(p[1], x);
	arr[2] = cubicInterpolate(p[2], x);
	arr[3] = cubicInterpolate(p[3], x);
	return cubicInterpolate(arr, y);
}

IMG *BMP_to_IMG(bitmap *bmp, int fno, int curss)
{
    int channel_mode = bmp->get_channel_mode();
    IMG *img = CreateImage(bmp->width(), bmp->height(), channel_mode);//(IMG *)g_os_malloc(sizeof(IMG));

    img->id = fno;

    if(curss > 0)
    {
        img->hh = (int)(curss / 3600);
        img->mm = (int)((curss % 3600) / 60);
        img->ss = (int)((curss % 3600) % 60);
    }
    else
    {
        img->hh = 0;
        img->mm = 0;
        img->ss = 0;
    }

    memcpy(img->buf, bmp->data(), img->area * img->channel * sizeof(unsigned char));

    return img;
}

IMG *BMP_to_IMG(bitmap *bmp)
{
    int channel_mode = bmp->get_channel_mode();
    IMG *img = CreateImage(bmp->width(), bmp->height(), channel_mode);//(IMG *)g_os_malloc(sizeof(IMG));

    img->id = 0;
    memcpy(img->buf, bmp->data(), img->area * img->channel * sizeof(unsigned char));

    return img;
}

bitmap *IMG_to_BMP(IMG *img)
{
    if(img->channel == 3)
    {
        bitmap *bmp = new bitmap(img->width,img->height,img->channel_mode);
        if(img->channel_mode == gray3_mode)
        {
            bmp->set_channel_mode(bgr_mode);
        }else
        {
            bmp->set_channel_mode(img->channel_mode);
        }
        memcpy(bmp->data(),img->buf, img->area * img->channel * sizeof(unsigned char));
        return bmp;
    }
    else if(img->channel == 1)
    {
        bitmap *bmp = new bitmap(img->width,img->height,bgr_mode);
        bmp->set_channel_mode(bgr_mode);

        unsigned char *dt = bmp->data();

        for(int i = 0 ; i < img->area ;i++ )
        {
            unsigned char v = img->buf[i];
            *(dt++) = v;
            *(dt++) = v;
            *(dt++) = v;
        }
        return bmp;
    }
    else
    {
        Logger("%s:%d\t cannot convert to BMP!!\n", __func__, __LINE__);
        assert(0);
    }
    return 0;
}

void Resample(unsigned char *a, unsigned char *b, int oldw, int oldh, int neww,  int newh,int use_cubic)
{
    int i;
    int j;
    int l;
    int c;
    double t;
    double u;
    double tmp;
    double d1, d2, d3, d4;
    u_char p1_b, p1_g, p1_r, p2_b, p2_g, p2_r, p3_b, p3_g, p3_r, p4_b, p4_g, p4_r;
    u_char red, green, blue;
    double p[4][4];
    for (i = 0; i < newh; i++) {
        for (j = 0; j < neww; j++) {

            tmp = (float) (i) / (float) (newh - 1) * (oldh - 1);
            l = (int) floor(tmp);
            if (l < 0) {
                l = 0;
            } else {
                if (l >= oldh - 1) {
                    l = oldh - 2;
                }
            }

            u = tmp - l;
            tmp = (float) (j) / (float) (neww - 1) * (oldw - 1);
            c = (int) floor(tmp);
            if (c < 0) {
                c = 0;
            } else {
                if (c >= oldw - 1) {
                    c = oldw - 2;
                }
            }
            t = tmp - c;

            if(use_cubic == 0)
            {
                /* coefficients */
                d1 = (1 - t) * (1 - u);
                d2 = t * (1 - u);
                d3 = t * u;
                d4 = (1 - t) * u;

                /* nearby pixels: a[i][j] */

                p1_b = *(a + ((l * oldw) + c) * 3 + 0);
                p1_g = *(a + ((l * oldw) + c) * 3 + 1);
                p1_r = *(a + ((l * oldw) + c) * 3 + 2);

                p2_b = *(a + ((l * oldw) + c + 1) * 3 + 0);
                p2_g = *(a + ((l * oldw) + c + 1) * 3 + 1);
                p2_r = *(a + ((l * oldw) + c + 1) * 3 + 2);

                p3_b = *(a + (((l+1) * oldw) + c + 1) * 3 + 0);
                p3_g = *(a + (((l+1) * oldw) + c + 1) * 3 + 1);
                p3_r = *(a + (((l+1) * oldw) + c + 1) * 3 + 2);

                p4_b = *(a + (((l+1) * oldw) + c) * 3 + 0);
                p4_g = *(a + (((l+1) * oldw) + c) * 3 + 1);
                p4_r = *(a + (((l+1) * oldw) + c) * 3 + 2);


                /* color components */
                blue  = p1_b * d1 + p2_b * d2 + p3_b * d3 + p4_b * d4;
                green = p1_g * d1 + p2_g * d2 + p3_g * d3 + p4_g * d4;
                red   = p1_r * d1 + p2_r * d2 + p3_r * d3 + p4_r * d4;

                /* new pixel R G B  */

                *( b + ( (i*neww) + j ) * 3 + 0) = blue;//blue;
                *( b + ( (i*neww) + j ) * 3 + 1) = green;
                *( b + ( (i*neww) + j ) * 3 + 2) = red;
            }
            else
            {
                //for 3 channel

                for(int z = 0 ; z < 3 ; z++)
                {
                    for(int zi = -1 ; zi < 3 ;zi++)
                    {
                        int zy = l+zi;
                        if(zy < 0) zy = 0;
                        else if(zy >= oldh) zy = oldh-1;
                        for(int zj = -1 ; zj < 3 ; zj++)
                        {
                            int zx = c+zj;
                            if(zx < 0) zx = 0;
                            else if(zx >= oldw) zx = oldw-1;

                            p[zi+1][zj+1] = *(a + ((zy * oldw) + zx) * 3 + z);
                        }
                    }

                    double v = bicubicInterpolate(p,t,u);
                    if(v < 0) v = 0;
                    if(v >= 256) v = 255;
                    *( b + ( (i*neww) + j ) * 3 + z) = (u_char) v;
                    //*( b + ( (i*neww) + j ) * 3 + z) = (u_char) p[1][1];
                }
            }
        }
    }
}

void ResizeImage(IMG *img, int width, int height, int use_cubic)
{
    if(img->width <= 0 ||img->height <= 0 ||
       width <= 0 || height <= 0 ||
       width > 2560 || height > 2560 )
        assert(0);

    unsigned char *new_buf = (unsigned char *)g_os_malloc(width*height*img->channel*sizeof(unsigned char));
    Resample(img->buf, new_buf, img->width, img->height, width, height, use_cubic);

    img->width = width;
    img->height = height;
    img->area = img->width * img->height;

    g_os_free(img->buf);

    img->buf = new_buf;

}

void LoadImage(IMG* img, char* filename)
{
    bitmap bmp(filename);
    image_drawer draw(bmp);
    img = BMP_to_IMG(&bmp);
}

void SaveImage(IMG* img, char* filename)
{
    bitmap *bmp = IMG_to_BMP(img);
    bmp->save_image(filename);
    delete(bmp);
    Logger("%s:%d\tsaveimage bmp done .. \n", __func__, __LINE__);
}

void SaveImagePNG(IMG* img, char* filename)
{
    ResizeImage(img, 640, 360, 0);

    unsigned char err;
    lodepng::State state;
    std::vector<unsigned char> ttin;
    std::vector<unsigned char> ttmid;
    std::vector<unsigned char> ttout;
    //const LodePNGCompressSettings lodepng_default_compress_settings = {2, 1, 4096, 3, 128, 1, 0, 0, 0};

    ttin.resize(img->width * img->height * 4);

    for(int i = 0; i < img->height; i ++)
    {
        for(int j = 0 ; j < img->width; j ++)
        {

            int addr = 3 * i * img->width + 3 * j;
            int newpos = 4 * i * img->width + 4 * j;
            ttin[newpos + 0] = img->buf[addr + 2];
            ttin[newpos + 1] = img->buf[addr + 1];
            ttin[newpos + 2] = img->buf[addr + 0];
            ttin[newpos + 3] = 255;
        }
    }

    err = lodepng::encode(ttmid, ttin, img->width, img->height);
    if(err)
    {
        std::cout << "encode error " << err << ": " << lodepng_error_text(err) << std::endl;
        return;
    }

    err = lodepng::save_file(ttmid, filename);
    if(err)
    {
        std::cout << "save error " << err << ": " << lodepng_error_text(err) << std::endl;
        return;
    }

}

int CheckTheImageGrayscale(IMG* img)
{
    int result = 0; //- 1 : grayscale  0: rgb
    int unit = 60;
    int ri = 0,  gi = 1, bi = 2;
    double r, g, b;
    bool br = false;

    int l1 = img->width *3;
    for(int y = img->height/3 ; y < img->height/3 + unit; y++)
    {
        for(int x = img->width/3; x < img->width/3 + unit ; x++)
        {
            r = img->buf[y*l1 + x * 3 + ri];
            g = img->buf[y*l1 + x * 3 + gi];
            b = img->buf[y*l1 + x * 3 + bi];

            if( r != g || g != b && b != r )
            {
//                printf(" rgb %.1f %.1f %.1f \n", r, g, b);
                br = true;
                break;
            }
        }

        if(br)
            break;
    }

    if(br)
        result = 0;
    else
        result = 1;

    return result;

}