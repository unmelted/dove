/*
 *****************************************************************************
 *                                                                           *
 *                          Platform Independent                             *
 *                    Bitmap Image Reader Writer Library                     *
 *                                                                           *
 * Author: Arash Partow - 2002                                               *
 * URL: http://partow.net/programming/bitmap/index.html                      *
 *                                                                           *
 * Note: This library only supports 24-bits per pixel bitmap format files.   *
 *                                                                           *
 * Copyright notice:                                                         *
 * Free use of the Platform Independent Bitmap Image Reader Writer Library   *
 * is permitted under the guidelines and in accordance with the most current *
 * version of the MIT License.                                               *
 * http://www.opensource.org/licenses/MIT                                    *
 *                                                                           *
 *****************************************************************************
*/


#ifndef INCLUDE_BITMAP_IMAGE_HPP
#define INCLUDE_BITMAP_IMAGE_HPP

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

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


struct rgb_t
{
    unsigned char   red;
    unsigned char green;
    unsigned char  blue;
};

class bitmap
{
public:

    bitmap();
    ~bitmap();
    bitmap(const std::string& filename);

    bitmap(const unsigned int width, const unsigned int height, int channel_mode);

    void clear();
    unsigned char* row(unsigned int row_index);
    void get_pixel(const unsigned int x, const unsigned int y,
                   unsigned char& red,
                   unsigned char& green,
                   unsigned char& blue);
    template <typename RGB>
    void get_pixel(const unsigned int x, const unsigned int y, RGB& colour);
    rgb_t get_pixel(const unsigned int x, const unsigned int y);

    void set_pixel(const unsigned int x, const unsigned int y,
                   const unsigned char red,
                   const unsigned char green,
                   const unsigned char blue);
    template <typename RGB>
    void set_pixel(const unsigned int x, const unsigned int y, const RGB& colour);

    bool copy_from(bitmap& image);
    bool copy_from(bitmap& source_image,
                   unsigned int& x_offset,
                   unsigned int& y_offset);

    unsigned int width();
    unsigned int height();
    void save_image(const std::string& file_name);
    void convert_to_grayscale();


    unsigned char* data();
    int get_channel_mode();
    void set_channel_mode(int mode);
    void bgr_to_rgb();
    void rgb_to_bgr();


private:
    void reverse_channels();
    const unsigned char* end();

   struct bitmap_file_header
   {
      unsigned short type;
      unsigned int   size;
      unsigned short reserved1;
      unsigned short reserved2;
      unsigned int   off_bits;

      unsigned int struct_size() const
      {
         return sizeof(type     ) +
                sizeof(size     ) +
                sizeof(reserved1) +
                sizeof(reserved2) +
                sizeof(off_bits ) ;
      }

      void clear()
      {
         std::memset(this, 0x00, sizeof(bitmap_file_header));
      }
   };

   struct bitmap_information_header
   {
      unsigned int   size;
      unsigned int   width;
      unsigned int   height;
      unsigned short planes;
      unsigned short bit_count;
      unsigned int   compression;
      unsigned int   size_image;
      unsigned int   x_pels_per_meter;
      unsigned int   y_pels_per_meter;
      unsigned int   clr_used;
      unsigned int   clr_important;

      unsigned int struct_size() const
      {
         return sizeof(size            ) +
                sizeof(width           ) +
                sizeof(height          ) +
                sizeof(planes          ) +
                sizeof(bit_count       ) +
                sizeof(compression     ) +
                sizeof(size_image      ) +
                sizeof(x_pels_per_meter) +
                sizeof(y_pels_per_meter) +
                sizeof(clr_used        ) +
                sizeof(clr_important   ) ;
      }

      void clear()
      {
         std::memset(this, 0x00, sizeof(bitmap_information_header));
      }
   };

    bool big_endian();

    unsigned short flip(const unsigned short& v);

    template <typename T>
    void read_from_stream(std::ifstream& stream,T& t);

    template <typename T>
    void write_to_stream(std::ofstream& stream,const T& t);

    void read_bfh(std::ifstream& stream, bitmap_file_header& bfh);
    void write_bfh(std::ofstream& stream, const bitmap_file_header& bfh);
    void read_bih(std::ifstream& stream,bitmap_information_header& bih);
    void write_bih(std::ofstream& stream, const bitmap_information_header& bih);

    std::size_t file_size(const std::string& file_name);
    void create_bitmap();

    void load_bitmap();
    unsigned int offset(const color_plane color);
   std::string  file_name_;
   unsigned int width_;
   unsigned int height_;
   unsigned int row_increment_;
   unsigned int bytes_per_pixel_;
   int channel_mode_;
   unsigned char *data_;
};

class image_drawer
{
public:
    image_drawer(bitmap& image);
    void rectangle(int x1, int y1, int x2, int y2);
    void triangle(int x1, int y1, int x2, int y2,int x3, int y3);
    void quadix(int x1, int y1, int x2, int y2,int x3, int y3, int x4, int y4);
    void line_segment(int x1, int y1, int x2, int y2);
    void plot_pen_pixel(int x, int y);
    void plot_pixel(int x, int y);
    void pen_width(unsigned int width);
    void pen_color(const unsigned char& red,
                   const unsigned char& green,
                   const unsigned char& blue);
    template <typename RGB>
    void pen_color(const RGB colour);

private:


    bitmap& image_;
    unsigned int  pen_width_;
    unsigned char pen_color_red_;
    unsigned char pen_color_green_;
    unsigned char pen_color_blue_;
};

#endif
