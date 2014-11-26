/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>

namespace scratch
{
//    typedef uint8_t u8;
//    typedef uint16_t u16;
//    typedef uint32_t u32;

    typedef uint8_t PixelR8;

    struct PixelRGB888
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    struct PixelRGBA8888
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };


    void PrintPixel(PixelR8 p)
    {
        std::cout << "[" << std::setw(3) << int(p) << "] ";
    }

    void PrintPixel(PixelRGB888 p)
    {
        std::cout << "["
                  << std::setw(3) << int(p.r) << ","
                  << std::setw(3) << int(p.g) << ","
                  << std::setw(3) << int(p.b)
                  << "] ";
    }

    template<typename Pixel>
    class Image
    {
    public:
        typedef typename std::vector<Pixel>::iterator PixelIterator;

    private:
        uint32_t const m_width;
        uint32_t const m_height;
        std::vector<Pixel> m_data;


        uint32_t col(PixelIterator it)
        {
            return std::distance(m_data.begin(),it)%m_width;
        }

        uint32_t row(PixelIterator it)
        {
            return std::distance(m_data.begin(),it)/m_width;
        }

    public:
        // initialize empty image
        Image(uint32_t width,
              uint32_t height) :
            m_width(width),
            m_height(height),
            m_data(width*height)
        {

        }

        Image(uint32_t width,
              uint32_t height,
              Pixel init_value) :
            m_width(width),
            m_height(height),
            m_data(width*height,init_value)
        {

        }

        Image(Image const &image_copy) :
            m_width(image_copy.width),
            m_height(image_copy.height),
            m_data(image_copy.data)
        {

        }

        uint32_t width() const
        {
            return m_width;
        }

        uint32_t height() const
        {
            return m_height;
        }

        PixelIterator at(uint32_t col, uint32_t row)
        {
            auto it = m_data.begin();
            std::advance(it,(row*m_width)+col);
            return it;
        }

        PixelIterator end()
        {
            return m_data.end();
        }

        PixelIterator begin()
        {
            return m_data.begin();
        }

        PixelIterator last()
        {
            auto it_last = m_data.end();
            std::advance(it_last,-1);
            return it_last;
        }

        Pixel const * data()
        {
            return &(m_data[0]);
        }

        //
        void insert(Image<Pixel> const &source,
                    PixelIterator source_it,
                    PixelIterator target_it)
        {
            insert(source,
                   source_it,
                   source.width(),
                   source.height(),
                   target_it);
        }

        void insert(Image<Pixel> const &source,
                    PixelIterator source_it,
                    uint32_t source_cols,
                    uint32_t source_rows,
                    PixelIterator target_it)
        {
            Image<Pixel> &target = (*this);

            uint32_t const overlap_rows =
                    std::min(target.height()-target.row(target_it),
                             source_rows);

            uint32_t const overlap_cols =
                    std::min(target.width()-target.col(target_it),
                             source_cols);

            // row by row
            for(uint32_t i=0; i < overlap_rows; i++) {
                auto source_row_end = source_it;
                std::advance(source_row_end,overlap_cols);
                std::copy(source_it,source_row_end,target_it);

                // advance source and target its by a row
                std::advance(source_it,source.width());
                std::advance(target_it,target.width());
            }
        }

        void print()
        {
            uint32_t pixel_index=0;
            for(auto &pixel : m_data) {
                PrintPixel(pixel);
                pixel_index++;
                if(pixel_index%m_width==0) {
                    std::cout << std::endl;
                }
            }
        }

    };
}

using namespace scratch;

int main()
{
    Image<PixelRGB888> bg(5,5,{0,0,0});
    Image<PixelRGB888> fg(5,1,{1,2,3});

    bg.insert(fg,fg.at(0,0),bg.at(0,4));
    bg.insert(bg,bg.at(0,4),3,1,bg.at(0,0));
    bg.print();

    return 0;
}
