#include "bitmap.hpp"
#include "os_api.hpp"
bitmap::bitmap()
{
    width_ = 0;
    height_ = 0;
    row_increment_ = 0;
    bytes_per_pixel_ = 3;
    channel_mode_ = bgr_mode;
    data_=0;
}

bitmap::~bitmap()
{
    if(data_) g_os_free(data_);
}

bitmap::bitmap(const std::string& filename)
{
    file_name_ = filename;
    width_ = 0;
    height_ = 0;
    data_=0;
    row_increment_ = 0;
    bytes_per_pixel_ = 3;
    channel_mode_ = bgr_mode;
    load_bitmap();
}

bitmap::bitmap(const unsigned int width, const unsigned int height, int channel_mode)
{
    width_ = width;
    height_ = height;
    data_=0;
    row_increment_ = 0;

    channel_mode_ = channel_mode;

    switch(channel_mode){
    case gray1_mode:
        bytes_per_pixel_ = 1;
        break;
    case rgb_mode:
    case bgr_mode:
    case gray3_mode:
        bytes_per_pixel_ = 3;
        break;
    case rgba_mode:
    case bgra_mode:
        bytes_per_pixel_ = 4;
        break;
    }
    create_bitmap();
}

void bitmap::clear()
{
    const unsigned char v = 0x00;

    unsigned char *tp=data();
    while(tp < end())
    {
        *tp++=v;
    }
}

unsigned char* bitmap::row(unsigned int row_index)
{
    return (unsigned char*)(&data_[(row_index * row_increment_)]);
}

void bitmap::get_pixel(const unsigned int x, const unsigned int y,
                         unsigned char& red,
                         unsigned char& green,
                         unsigned char& blue)
{
    const unsigned int y_offset = y * row_increment_;
    const unsigned int x_offset = x * bytes_per_pixel_;
    const unsigned int offset   = y_offset + x_offset;

    blue  = data_[offset + 0];
    green = data_[offset + 1];
    red   = data_[offset + 2];
}

template <typename RGB>
void bitmap::get_pixel(const unsigned int x, const unsigned int y, RGB& colour)
{
    get_pixel(x, y, colour.red, colour.green, colour.blue);
}


rgb_t bitmap::get_pixel(const unsigned int x, const unsigned int y)
{
    rgb_t colour;
    get_pixel(x, y, colour.red, colour.green, colour.blue);
    return colour;
}


void bitmap::set_pixel(const unsigned int x, const unsigned int y,
               const unsigned char red,
               const unsigned char green,
               const unsigned char blue)
{
    const unsigned int y_offset = y * row_increment_;
    const unsigned int x_offset = x * bytes_per_pixel_;
    const unsigned int offset   = y_offset + x_offset;

    data_[offset + 0] = blue;
    data_[offset + 1] = green;
    data_[offset + 2] = red;
}

template <typename RGB>
void bitmap::set_pixel(const unsigned int x, const unsigned int y, const RGB& colour)
{
    set_pixel(x, y, colour.red, colour.green, colour.blue);
}

bool bitmap::copy_from(bitmap& image)
{
    if (
        (image.height_ != height_) ||
        (image.width_  != width_ )
        )
    {
        return false;
    }

    data_ = image.data_;

    return true;
}

bool bitmap::copy_from(bitmap& source_image,
               unsigned int& x_offset,
               unsigned int& y_offset)
{
      if ((x_offset + source_image.width_ ) > width_ ) { return false; }
      if ((y_offset + source_image.height_) > height_) { return false; }

      for (unsigned int y = 0; y < source_image.height_; ++y)
      {
          unsigned char* itr1           = row(y + y_offset) + x_offset * bytes_per_pixel_;
          unsigned char* itr2     = source_image.row(y);
          unsigned char* itr2_end = itr2 + source_image.width_ * bytes_per_pixel_;

         std::copy(itr2, itr2_end, itr1);
      }

      return true;
}

unsigned int bitmap::width()
{
    return width_;
}

unsigned int bitmap::height()
{
    return height_;
}

void bitmap::save_image(const std::string& file_name)
{
    std::ofstream stream(file_name.c_str(),std::ios::binary);

    if (!stream)
    {
        std::cerr << "bitmap::save_image(): Error - Could not open file "
                  << file_name << " for writing!" << std::endl;
        return;
    }

    bitmap_information_header bih;

    bih.width            = width_;
    bih.height           = height_;
    bih.bit_count        = static_cast<unsigned short>(bytes_per_pixel_ << 3);
    bih.clr_important    = 0;
    bih.clr_used         = 0;
    bih.compression      = 0;
    bih.planes           = 1;
    bih.size             = bih.struct_size();
    bih.x_pels_per_meter = 0;
    bih.y_pels_per_meter = 0;
    bih.size_image       = (((bih.width * bytes_per_pixel_) + 3) & 0x0000FFFC) * bih.height;

    bitmap_file_header bfh;

    bfh.type             = 19778;
    bfh.size             = bfh.struct_size() + bih.struct_size() + bih.size_image;
    bfh.reserved1        = 0;
    bfh.reserved2        = 0;
    bfh.off_bits         = bih.struct_size() + bfh.struct_size();

    write_bfh(stream,bfh);
    write_bih(stream,bih);

    unsigned int padding = (4 - ((3 * width_) % 4)) % 4;
    char padding_data[4] = { 0x00, 0x00, 0x00, 0x00 };

    for (unsigned int i = 0; i < height_; ++i)
    {
        const unsigned char* data_ptr = &data_[(row_increment_ * (height_ - i - 1))];

        stream.write(reinterpret_cast<const char*>(data_ptr), sizeof(unsigned char) * bytes_per_pixel_ * width_);
        stream.write(padding_data,padding);
    }

    stream.close();
}

void bitmap::convert_to_grayscale()
{
    double r_scaler = 0.299;
    double g_scaler = 0.587;
    double b_scaler = 0.114;

    if ( bytes_per_pixel_ != 3)
    {
        printf("get not convert current mode is %d\n",channel_mode_);
        return;
    }
    if (rgb_mode == channel_mode_)
    {
        std::swap(r_scaler, b_scaler);
    }

    for (unsigned char* itr = data(); itr < end(); )
    {
        unsigned char gray_value = static_cast<unsigned char>
            (
                (r_scaler * (*(itr + 2))) +
                (g_scaler * (*(itr + 1))) +
                (b_scaler * (*(itr + 0)))
                );

        *(itr++) = gray_value;
        *(itr++) = gray_value;
        *(itr++) = gray_value;
    }
}

unsigned char*  bitmap::data()
{
    return data_;
}

void bitmap::reverse_channels()
{
    if (3 != bytes_per_pixel_)
        return;

    for (unsigned char* itr = data(); itr < end(); itr += bytes_per_pixel_)
    {
        std::swap(*(itr + 0),*(itr + 2));
    }
}

int bitmap::get_channel_mode()
{
    return channel_mode_;
}

void bitmap::set_channel_mode(int mode)
{
    channel_mode_ = mode;
}

void bitmap::bgr_to_rgb()
{
    if ((bgr_mode == channel_mode_) && (3 == bytes_per_pixel_))
    {
        reverse_channels();
        channel_mode_ = rgb_mode;
    }
}

void bitmap::rgb_to_bgr()
{
    if ((rgb_mode == channel_mode_) && (3 == bytes_per_pixel_))
    {
        reverse_channels();
        channel_mode_ = bgr_mode;
    }
}

unsigned int bitmap::offset(const color_plane color)
{
    switch (channel_mode_)
    {
    case rgb_mode : {
        switch (color)
        {
        case red_plane   : return 0;
        case green_plane : return 1;
        case blue_plane  : return 2;
        default          : return std::numeric_limits<unsigned int>::max();
        }
    }

    case bgr_mode : {
        switch (color)
        {
        case red_plane   : return 2;
        case green_plane : return 1;
        case blue_plane  : return 0;
        default          : return std::numeric_limits<unsigned int>::max();
        }
    }

    default       : return std::numeric_limits<unsigned int>::max();
    }
}

const unsigned char* bitmap::end()
{
    return const_cast<unsigned char*>(data() + (width_*height_*3));
}

bool bitmap::big_endian()
{
    unsigned int v = 0x01;

    return (1 != reinterpret_cast<char*>(&v)[0]);
}

unsigned short bitmap::flip(const unsigned short& v)
{
    return ((v >> 8) | (v << 8));
}

template <typename T>
void bitmap::read_from_stream(std::ifstream& stream,T& t)
{
    stream.read(reinterpret_cast<char*>(&t),sizeof(T));
}

template <typename T>
void bitmap::write_to_stream(std::ofstream& stream,const T& t)
{
    stream.write(reinterpret_cast<const char*>(&t),sizeof(T));
}

void bitmap::read_bfh(std::ifstream& stream, bitmap_file_header& bfh)
{
    read_from_stream(stream,bfh.type     );
    read_from_stream(stream,bfh.size     );
    read_from_stream(stream,bfh.reserved1);
    read_from_stream(stream,bfh.reserved2);
    read_from_stream(stream,bfh.off_bits );

    if (big_endian())
    {
        bfh.type      = flip(bfh.type     );
        bfh.size      = flip(bfh.size     );
        bfh.reserved1 = flip(bfh.reserved1);
        bfh.reserved2 = flip(bfh.reserved2);
        bfh.off_bits  = flip(bfh.off_bits );
    }
}

void bitmap::write_bfh(std::ofstream& stream, const bitmap_file_header& bfh)
{
    if (big_endian())
    {
        write_to_stream(stream,flip(bfh.type     ));
        write_to_stream(stream,flip(bfh.size     ));
        write_to_stream(stream,flip(bfh.reserved1));
        write_to_stream(stream,flip(bfh.reserved2));
        write_to_stream(stream,flip(bfh.off_bits ));
    }
    else
    {
        write_to_stream(stream,bfh.type     );
        write_to_stream(stream,bfh.size     );
        write_to_stream(stream,bfh.reserved1);
        write_to_stream(stream,bfh.reserved2);
        write_to_stream(stream,bfh.off_bits );
    }
}

void bitmap::read_bih(std::ifstream& stream,bitmap_information_header& bih)
{
      read_from_stream(stream,bih.size            );
      read_from_stream(stream,bih.width           );
      read_from_stream(stream,bih.height          );
      read_from_stream(stream,bih.planes          );
      read_from_stream(stream,bih.bit_count       );
      read_from_stream(stream,bih.compression     );
      read_from_stream(stream,bih.size_image      );
      read_from_stream(stream,bih.x_pels_per_meter);
      read_from_stream(stream,bih.y_pels_per_meter);
      read_from_stream(stream,bih.clr_used        );
      read_from_stream(stream,bih.clr_important   );

      if (big_endian())
      {
         bih.size          = flip(bih.size               );
         bih.width         = flip(bih.width              );
         bih.height        = flip(bih.height             );
         bih.planes        = flip(bih.planes             );
         bih.bit_count     = flip(bih.bit_count          );
         bih.compression   = flip(bih.compression        );
         bih.size_image    = flip(bih.size_image         );
         bih.x_pels_per_meter = flip(bih.x_pels_per_meter);
         bih.y_pels_per_meter = flip(bih.y_pels_per_meter);
         bih.clr_used      = flip(bih.clr_used           );
         bih.clr_important = flip(bih.clr_important      );
      }
   }

void bitmap::write_bih(std::ofstream& stream, const bitmap_information_header& bih)
{
    if (big_endian())
    {
        write_to_stream(stream,flip(bih.size            ));
        write_to_stream(stream,flip(bih.width           ));
        write_to_stream(stream,flip(bih.height          ));
        write_to_stream(stream,flip(bih.planes          ));
        write_to_stream(stream,flip(bih.bit_count       ));
        write_to_stream(stream,flip(bih.compression     ));
        write_to_stream(stream,flip(bih.size_image      ));
        write_to_stream(stream,flip(bih.x_pels_per_meter));
        write_to_stream(stream,flip(bih.y_pels_per_meter));
        write_to_stream(stream,flip(bih.clr_used        ));
        write_to_stream(stream,flip(bih.clr_important   ));
    }
    else
    {
        write_to_stream(stream,bih.size            );
        write_to_stream(stream,bih.width           );
        write_to_stream(stream,bih.height          );
        write_to_stream(stream,bih.planes          );
        write_to_stream(stream,bih.bit_count       );
        write_to_stream(stream,bih.compression     );
        write_to_stream(stream,bih.size_image      );
        write_to_stream(stream,bih.x_pels_per_meter);
        write_to_stream(stream,bih.y_pels_per_meter);
        write_to_stream(stream,bih.clr_used        );
        write_to_stream(stream,bih.clr_important   );
    }
}

std::size_t bitmap::file_size(const std::string& file_name)
{
    std::ifstream file(file_name.c_str(),std::ios::in | std::ios::binary);
    if (!file) return 0;
    file.seekg (0, std::ios::end);
    return static_cast<std::size_t>(file.tellg());
}


void bitmap::create_bitmap()
{
    row_increment_ = width_ * bytes_per_pixel_;
    data_ = (unsigned char *)g_os_calloc(sizeof(unsigned char), height_ * row_increment_);
    //data_.resize(height_ * row_increment_);
}

void bitmap::load_bitmap()
{
    std::ifstream stream(file_name_.c_str(),std::ios::binary);

    if (!stream)
    {
        std::cerr << "bitmap::load_bitmap() ERROR: bitmap - "
                  << "file " << file_name_ << " not found!" << std::endl;
        return;
    }

    width_  = 0;
    height_ = 0;

    bitmap_file_header bfh;
    bitmap_information_header bih;

    bfh.clear();
    bih.clear();

    read_bfh(stream,bfh);
    read_bih(stream,bih);

    if (bfh.type != 19778)
    {
        std::cerr << "bitmap::load_bitmap() ERROR: bitmap - "
                  << "Invalid type value " << bfh.type << " expected 19778." << std::endl;

        bfh.clear();
        bih.clear();

        stream.close();

        return;
    }

    if (bih.bit_count != 24)
    {
        std::cerr << "bitmap::load_bitmap() ERROR: bitmap - "
                  << "Invalid bit depth " << bih.bit_count << " expected 24." << std::endl;

        bfh.clear();
        bih.clear();

        stream.close();

        return;
    }

    if (bih.size != bih.struct_size())
    {
        std::cerr << "bitmap::load_bitmap() ERROR: bitmap - "
                  << "Invalid BIH size " << bih.size
                  << " expected " << bih.struct_size() << std::endl;

        bfh.clear();
        bih.clear();

        stream.close();

        return;
    }

    width_  = bih.width;
    height_ = bih.height;

    bytes_per_pixel_ = bih.bit_count >> 3;

    unsigned int padding = (4 - ((3 * width_) % 4)) % 4;
    char padding_data[4] = { 0x00, 0x00, 0x00, 0x00 };

    std::size_t bitmap_file_size = file_size(file_name_);

    std::size_t bitmap_logical_size = (height_ * width_ * bytes_per_pixel_) +
        (height_ * padding)                   +
        bih.struct_size()                    +
        bfh.struct_size()                    ;

    if (bitmap_file_size != bitmap_logical_size)
    {
        std::cerr << "bitmap::load_bitmap() ERROR: bitmap - "
                  << "Mismatch between logical and physical sizes of bitmap. "
                  << "Logical: "  << bitmap_logical_size << " "
                  << "Physical: " << bitmap_file_size    << std::endl;

        bfh.clear();
        bih.clear();

        stream.close();

        return;
    }

    create_bitmap();

    for (unsigned int i = 0; i < height_; ++i)
    {
        unsigned char* data_ptr = row(height_ - i - 1); // read in inverted row order

        stream.read(reinterpret_cast<char*>(data_ptr), sizeof(char) * bytes_per_pixel_ * width_);
        stream.read(padding_data,padding);
    }
}

image_drawer::image_drawer(bitmap& image)
    :image_(image)
{
    pen_width_ = 1;
    pen_color_red_ = 0;
    pen_color_green_ = 0;
    pen_color_blue_ = 0;
}

void image_drawer::rectangle(int x1, int y1, int x2, int y2)
{
    line_segment(x1, y1, x2, y1);
    line_segment(x2, y1, x2, y2);
    line_segment(x2, y2, x1, y2);
    line_segment(x1, y2, x1, y1);
}

void image_drawer::triangle(int x1, int y1, int x2, int y2,int x3, int y3)
{
    line_segment(x1, y1, x2, y2);
    line_segment(x2, y2, x3, y3);
    line_segment(x3, y3, x1, y1);
}

void image_drawer::quadix(int x1, int y1, int x2, int y2,int x3, int y3, int x4, int y4)
{
    line_segment(x1, y1, x2, y2);
    line_segment(x2, y2, x3, y3);
    line_segment(x3, y3, x4, y4);
    line_segment(x4, y4, x1, y1);
}

void image_drawer::line_segment(int x1, int y1, int x2, int y2)
{
    int steep = 0;
    int sx    = ((x2 - x1) > 0) ? 1 : -1;
    int sy    = ((y2 - y1) > 0) ? 1 : -1;
    int dx    = abs(x2 - x1);
    int dy    = abs(y2 - y1);

    if (dy > dx)
    {
        std::swap(x1,y1);
        std::swap(dx,dy);
        std::swap(sx,sy);

        steep = 1;
    }

    int e = 2 * dy - dx;

    for (int i = 0; i < dx; ++i)
    {
        if (steep)
            plot_pen_pixel(y1,x1);
        else
            plot_pen_pixel(x1,y1);

        while (e >= 0)
        {
            y1 += sy;
            e -= (dx << 1);
        }

        x1 += sx;
        e  += (dy << 1);
    }

    plot_pen_pixel(x2,y2);
}

#if 0
void horiztonal_line_segment(int x1, int x2, int y)
{
    if (x1 > x2)
    {
        std::swap(x1,x2);
    }

    for (int i = 0; i < (x2 - x1); ++i)
    {
        plot_pen_pixel(x1 +  i,y);
    }
}

void vertical_line_segment(int y1, int y2, int x)
{
    if (y1 > y2)
    {
        std::swap(y1,y2);
    }

    for (int i = 0; i < (y2 - y1); ++i)
    {
        plot_pen_pixel(x, y1 +  i);
    }
}

void ellipse(int centerx, int centery, int a, int b)
{
    int t1 = a * a;
    int t2 = t1 << 1;
    int t3 = t2 << 1;
    int t4 = b * b;
    int t5 = t4 << 1;
    int t6 = t5 << 1;
    int t7 = a * t5;
    int t8 = t7 << 1;
    int t9 = 0;

    int d1 = t2 - t7 + (t4 >> 1);
    int d2 = (t1 >> 1) - t8 + t5;
    int x  = a;
    int y  = 0;

    int negative_tx = centerx - x;
    int positive_tx = centerx + x;
    int negative_ty = centery - y;
    int positive_ty = centery + y;

    while (d2 < 0)
    {
        plot_pen_pixel(positive_tx, positive_ty);
        plot_pen_pixel(positive_tx, negative_ty);
        plot_pen_pixel(negative_tx, positive_ty);
        plot_pen_pixel(negative_tx, negative_ty);

        ++y;

        t9 = t9 + t3;

        if (d1 < 0)
        {
            d1 = d1 + t9 + t2;
            d2 = d2 + t9;
        }
        else
        {
            x--;
            t8 = t8 - t6;
            d1 = d1 + (t9 + t2 - t8);
            d2 = d2 + (t9 + t5 - t8);
            negative_tx = centerx - x;
            positive_tx = centerx + x;
        }

        negative_ty = centery - y;
        positive_ty = centery + y;
    }

    do
    {
        plot_pen_pixel(positive_tx, positive_ty);
        plot_pen_pixel(positive_tx, negative_ty);
        plot_pen_pixel(negative_tx, positive_ty);
        plot_pen_pixel(negative_tx, negative_ty);

        x--;
        t8 = t8 - t6;

        if (d2 < 0)
        {
            ++y;
            t9 = t9 + t3;
            d2 = d2 + (t9 + t5 - t8);
            negative_ty = centery - y;
            positive_ty = centery + y;
        }
        else
            d2 = d2 + (t5 - t8);

        negative_tx = centerx - x;
        positive_tx = centerx + x;
    }
    while (x >= 0);
}

void circle(int centerx, int centery, int radius)
{
    int x = 0;
    int d = (1 - radius) << 1;

    while (radius >= 0)
    {
        plot_pen_pixel(centerx + x, centery + radius);
        plot_pen_pixel(centerx + x, centery - radius);
        plot_pen_pixel(centerx - x, centery + radius);
        plot_pen_pixel(centerx - x, centery - radius);

        if ((d + radius) > 0)
            d -= ((--radius) << 1) - 1;
        if (x > d)
            d += ((++x) << 1) + 1;
    }
}
#endif
void image_drawer::plot_pen_pixel(int x, int y)
{
    switch (pen_width_)
    {
    case 1  : plot_pixel(x,y);
        break;

    case 2  : {
        plot_pixel(x    , y    );
        plot_pixel(x + 1, y    );
        plot_pixel(x + 1, y + 1);
        plot_pixel(x    , y + 1);
    }
        break;

    case  3 : {
        plot_pixel(x    , y - 1);
        plot_pixel(x - 1, y - 1);
        plot_pixel(x + 1, y - 1);

        plot_pixel(x    , y    );
        plot_pixel(x - 1, y    );
        plot_pixel(x + 1, y    );

        plot_pixel(x    , y + 1);
        plot_pixel(x - 1, y + 1);
        plot_pixel(x + 1, y + 1);
    }
        break;

    default : plot_pixel(x,y);
        break;
    }
}

void image_drawer::plot_pixel(int x, int y)
{
    if (
        (x < 0) ||
        (y < 0) ||
        (x >= static_cast<int>(image_.width ())) ||
        (y >= static_cast<int>(image_.height()))
        )
        return;

    image_.set_pixel(x,y,pen_color_red_,pen_color_green_,pen_color_blue_);
}

void image_drawer::pen_width(unsigned int width)
{
    if ((width > 0) && (width < 4))
    {
        pen_width_ = width;
    }
}

void image_drawer::pen_color(const unsigned char& red,
               const unsigned char& green,
               const unsigned char& blue)
{
    pen_color_red_   = red;
    pen_color_green_ = green;
    pen_color_blue_  = blue;
}

template <typename RGB>
void image_drawer::pen_color(const RGB colour)
{
    pen_color_red_   = colour.red;
    pen_color_green_ = colour.green;
    pen_color_blue_  = colour.blue;
}
