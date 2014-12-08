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

#ifndef SCRATCH_INLINE_IMAGE_H
#define SCRATCH_INLINE_IMAGE_H

// sys
#include <cstdint>

// stl
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <cassert>
#include <limits>

namespace ilim
{
    // common pixel types
    struct R8
    {
        uint8_t r;
    };

    struct R16
    {
        uint16_t r;
    };

    struct R32
    {
        uint32_t r;
    };

    struct RGB555 // TODO check memory layout
    {
        uint8_t r : 5;
        uint8_t g : 5;
        uint8_t b : 5;
        uint8_t : 1; // fill to 16 bits
    };

    struct RGB565 // TODO check memory layout
    {
        uint8_t r : 5;
        uint8_t g : 6;
        uint8_t b : 5;
    };

    struct RGB8
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    struct RGBA8
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };

    struct RGBA16
    {
        uint16_t r;
        uint16_t g;
        uint16_t b;
        uint16_t a;
    };

    struct RGB32F
    {
        float r;
        float g;
        float b;
    };

    struct RGBA32F
    {
        float r;
        float g;
        float b;
        float a;
    };

    struct RGBA64F
    {
        double r;
        double g;
        double b;
        double a;
    };

    // ============================================================= //

    namespace ilim_detail
    {
        // compile-time int pow // VERIFY its compile time
        constexpr uint64_t ct_ui_pow(uint64_t base, uint64_t exp)
        {
            return ((exp == 0) ? 1 : base*ct_ui_pow(base,exp-1));
        }

        // ============================================================= //

        template <typename Pixel>
        struct pixel_traits
        {
            static const uint8_t channel_count = 0;

            static const bool is_int_type = false;
            static const bool single_bitdepth = false;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };

        // ============================================================= //

        // common pixel traits

        template<>
        struct pixel_traits<R8>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 8;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };

        template<>
        struct pixel_traits<R16>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 16;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };

        template<>
        struct pixel_traits<R32>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 32;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<RGB555>
        {
            static const uint8_t channel_count = 3;

            static const bool is_int_type = true;
            static const bool single_bitdepth = false;

            static const uint8_t bits_r = 5;
            static const uint8_t bits_g = 5;
            static const uint8_t bits_b = 5;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<RGB565>
        {
            static const uint8_t channel_count = 3;

            static const bool is_int_type = true;
            static const bool single_bitdepth = false;

            static const uint8_t bits_r = 5;
            static const uint8_t bits_g = 6;
            static const uint8_t bits_b = 5;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<RGB8>
        {
            static const uint8_t channel_count = 3;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 8;
            static const uint8_t bits_g = 8;
            static const uint8_t bits_b = 8;
            static const uint8_t bits_a = 0;
        };

        template<>
        struct pixel_traits<RGBA8>
        {
            static const uint8_t channel_count = 4;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 8;
            static const uint8_t bits_g = 8;
            static const uint8_t bits_b = 8;
            static const uint8_t bits_a = 8;
        };

        template<>
        struct pixel_traits<RGBA16>
        {
            static const uint8_t channel_count = 4;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 16;
            static const uint8_t bits_g = 16;
            static const uint8_t bits_b = 16;
            static const uint8_t bits_a = 16;
        };

        template<>
        struct pixel_traits<RGB32F>
        {
            static const uint8_t channel_count = 3;

            static const bool is_int_type = false;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 32;
            static const uint8_t bits_g = 32;
            static const uint8_t bits_b = 32;
            static const uint8_t bits_a = 0;
        };

        template<>
        struct pixel_traits<RGBA32F>
        {
            static const uint8_t channel_count = 4;

            static const bool is_int_type = false;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 32;
            static const uint8_t bits_g = 32;
            static const uint8_t bits_b = 32;
            static const uint8_t bits_a = 32;
        };

        template<>
        struct pixel_traits<RGBA64F>
        {
            static const uint8_t channel_count = 4;

            static const bool is_int_type = false;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 64;
            static const uint8_t bits_g = 64;
            static const uint8_t bits_b = 64;
            static const uint8_t bits_a = 64;
        };

        // ============================================================= //

        enum class assign_mode {
            no_op,
            sub,
            int_to_float,
            float_to_int,
            float_to_float,
            upscale,
            downscale
        };

        // ============================================================= //

        template<assign_mode mode>
        struct channel_r;

        // no-op: dst channel dne
        template<>
        struct channel_r<assign_mode::no_op>
        {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &)
            {
                // no op
            }
        };

        template<>
        struct channel_r<assign_mode::sub> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &dst) {
                dst.r = 0;
            }
        };

        template<>
        struct channel_r<assign_mode::int_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelDst::r) max = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_r)-1;
                dst.r = src.r/max;
            }
        };

        template<>
        struct channel_r<assign_mode::float_to_int> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr auto max = ct_ui_pow(2,pixel_traits<PixelDst>::bits_r)-1;
                dst.r = src.r*max;
            }
        };

        template<>
        struct channel_r<assign_mode::float_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                dst.r = src.r;
            }
        };

        template<>
        struct channel_r<assign_mode::upscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelSrc::r) us_div = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_r)-1;
                constexpr decltype(PixelDst::r) us_mul = ct_ui_pow(2,pixel_traits<PixelDst>::bits_r)-1;
                dst.r = (us_mul/us_div)*src.r;
            }
        };

        template<>
        struct channel_r<assign_mode::downscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr uint8_t ds_shift = pixel_traits<PixelSrc>::bits_r-pixel_traits<PixelDst>::bits_r;
                dst.r = (src.r >> ds_shift);
            }
        };

        template<typename PixelSrc,typename PixelDst>
        void assign_r(PixelSrc const &src, PixelDst &dst)
        {
            typedef pixel_traits<PixelSrc> src_traits;
            typedef pixel_traits<PixelDst> dst_traits;

            constexpr assign_mode mode = (
                        (dst_traits::bits_r==0) ? assign_mode::no_op :
                        (src_traits::bits_r==0) ? assign_mode::sub :
                        (src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::int_to_float :
                        (!src_traits::is_int_type && dst_traits::is_int_type) ? assign_mode::float_to_int :
                        (!src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::float_to_float :
                        (dst_traits::bits_r > src_traits::bits_r) ? assign_mode::upscale : assign_mode::downscale);

//            std::cout << "mode: " << src_traits::is_int_type << ", " << dst_traits::is_int_type << std::endl;

            channel_r<mode>::assign(src,dst);
        }

        // ============================================================= //

        template<assign_mode mode>
        struct channel_g;

        // no-op: dst channel dne
        template<>
        struct channel_g<assign_mode::no_op>
        {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &)
            {
                // no op
            }
        };

        template<>
        struct channel_g<assign_mode::sub> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &dst) {
                dst.g = 0;
            }
        };

        template<>
        struct channel_g<assign_mode::int_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelDst::g) max = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_g)-1;
                dst.g = src.g/max;
            }
        };

        template<>
        struct channel_g<assign_mode::float_to_int> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr auto max = ct_ui_pow(2,pixel_traits<PixelDst>::bits_g)-1;
                dst.g = src.g*max;
            }
        };

        template<>
        struct channel_g<assign_mode::float_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                dst.g = src.g;
            }
        };

        template<>
        struct channel_g<assign_mode::upscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelSrc::g) us_div = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_g)-1;
                constexpr decltype(PixelDst::g) us_mul = ct_ui_pow(2,pixel_traits<PixelDst>::bits_g)-1;
                dst.g = (us_mul/us_div)*src.g;
            }
        };

        template<>
        struct channel_g<assign_mode::downscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr uint8_t ds_shift = pixel_traits<PixelSrc>::bits_g-pixel_traits<PixelDst>::bits_g;
                dst.g = (src.g >> ds_shift);
            }
        };

        template<typename PixelSrc,typename PixelDst>
        void assign_g(PixelSrc const &src, PixelDst &dst)
        {
            typedef pixel_traits<PixelSrc> src_traits;
            typedef pixel_traits<PixelDst> dst_traits;

            constexpr assign_mode mode = (
                        (dst_traits::bits_g==0) ? assign_mode::no_op :
                        (src_traits::bits_g==0) ? assign_mode::sub :
                        (src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::int_to_float :
                        (!src_traits::is_int_type && dst_traits::is_int_type) ? assign_mode::float_to_int :
                        (!src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::float_to_float :
                        (dst_traits::bits_g > src_traits::bits_g) ? assign_mode::upscale : assign_mode::downscale);

            channel_g<mode>::assign(src,dst);
        }

        // ============================================================= //

        template<assign_mode mode>
        struct channel_b;

        // no-op: dst channel dne
        template<>
        struct channel_b<assign_mode::no_op>
        {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &)
            {
                // no op
            }
        };

        template<>
        struct channel_b<assign_mode::sub> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &dst) {
                dst.b = 0;
            }
        };

        template<>
        struct channel_b<assign_mode::int_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelDst::b) max = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_b)-1;
                dst.b = src.b/max;
            }
        };

        template<>
        struct channel_b<assign_mode::float_to_int> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr auto max = ct_ui_pow(2,pixel_traits<PixelDst>::bits_b)-1;
                dst.b = src.b*max;
            }
        };

        template<>
        struct channel_b<assign_mode::float_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                dst.b = src.b;
            }
        };

        template<>
        struct channel_b<assign_mode::upscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelSrc::b) us_div = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_b)-1;
                constexpr decltype(PixelDst::b) us_mul = ct_ui_pow(2,pixel_traits<PixelDst>::bits_b)-1;
                dst.b = (us_mul/us_div)*src.b;
            }
        };

        template<>
        struct channel_b<assign_mode::downscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr uint8_t ds_shift = pixel_traits<PixelSrc>::bits_b-pixel_traits<PixelDst>::bits_b;
                dst.b = (src.b >> ds_shift);
            }
        };

        template<typename PixelSrc,typename PixelDst>
        void assign_b(PixelSrc const &src, PixelDst &dst)
        {
            typedef pixel_traits<PixelSrc> src_traits;
            typedef pixel_traits<PixelDst> dst_traits;

            constexpr assign_mode mode = (
                        (dst_traits::bits_b==0) ? assign_mode::no_op :
                        (src_traits::bits_b==0) ? assign_mode::sub :
                        (src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::int_to_float :
                        (!src_traits::is_int_type && dst_traits::is_int_type) ? assign_mode::float_to_int :
                        (!src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::float_to_float :
                        (dst_traits::bits_b > src_traits::bits_b) ? assign_mode::upscale : assign_mode::downscale);

            channel_b<mode>::assign(src,dst);
        }

        // ============================================================= //

        template<assign_mode mode>
        struct channel_a;

        // no-op: dst channel dne
        template<>
        struct channel_a<assign_mode::no_op>
        {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &)
            {
                // no op
            }
        };

        template<>
        struct channel_a<assign_mode::sub> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &, PixelDst &dst) {
                dst.a = 1;
            }
        };

        template<>
        struct channel_a<assign_mode::int_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelDst::a) max = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_a)-1;
                dst.a = src.a/max;
            }
        };

        template<>
        struct channel_a<assign_mode::float_to_int> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr auto max = ct_ui_pow(2,pixel_traits<PixelDst>::bits_a)-1;
                dst.a = src.a*max;
            }
        };

        template<>
        struct channel_a<assign_mode::float_to_float> {
            template <typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                dst.a = src.a;
            }
        };

        template<>
        struct channel_a<assign_mode::upscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr decltype(PixelSrc::a) us_div = ct_ui_pow(2,pixel_traits<PixelSrc>::bits_a)-1;
                constexpr decltype(PixelDst::a) us_mul = ct_ui_pow(2,pixel_traits<PixelDst>::bits_a)-1;
                dst.a = (us_mul/us_div)*src.a;
            }
        };

        template<>
        struct channel_a<assign_mode::downscale> {
            template<typename PixelSrc, typename PixelDst>
            static void assign(PixelSrc const &src, PixelDst &dst) {
                constexpr uint8_t ds_shift = pixel_traits<PixelSrc>::bits_a-pixel_traits<PixelDst>::bits_a;
                dst.a = (src.a >> ds_shift);
            }
        };

        template<typename PixelSrc,typename PixelDst>
        void assign_a(PixelSrc const &src, PixelDst &dst)
        {
            typedef pixel_traits<PixelSrc> src_traits;
            typedef pixel_traits<PixelDst> dst_traits;

            constexpr assign_mode mode = (
                        (dst_traits::bits_a==0) ? assign_mode::no_op :
                        (src_traits::bits_a==0) ? assign_mode::sub :
                        (src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::int_to_float :
                        (!src_traits::is_int_type && dst_traits::is_int_type) ? assign_mode::float_to_int :
                        (!src_traits::is_int_type && !dst_traits::is_int_type) ? assign_mode::float_to_float :
                        (dst_traits::bits_a > src_traits::bits_a) ? assign_mode::upscale : assign_mode::downscale);

            channel_a<mode>::assign(src,dst);
        }

        // ============================================================= //

        template <typename PixelSrc,typename PixelDst>
        void conv_pixels(std::vector<PixelSrc> const &list_src,
                         std::vector<PixelDst> &list_dst)
        {
            list_dst.clear();
            list_dst.reserve(list_src.size());

            // int -> int
            for(auto const &src : list_src) {
                PixelDst dst;
                assign_r(src,dst);
                assign_g(src,dst);
                assign_b(src,dst);
                assign_a(src,dst);

                list_dst.push_back(dst);
            }
        }

        // ref: http://stackoverflow.com/questions/105252/...
        // ...how-do-i-convert-between-big-endian-and-little-endian-values-in-c/105297#105297
        template <typename T>
        T swap_endian(T u)
        {
            union
            {
                T u;
                unsigned char u8[sizeof(T)];
            } source, dest;

            source.u = u;

            for (size_t k = 0; k < sizeof(T); k++)
                dest.u8[k] = source.u8[sizeof(T) - k - 1];

            return dest.u;
        }

    } // detail

    // ============================================================= //

    template<typename Pixel>
    class Image
    {
    public:
        typedef typename std::vector<Pixel>::iterator PixelIterator;

    private:
        uint32_t m_width;
        uint32_t m_height;
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
        // initialize null image
        Image() :
            m_width(0),
            m_height(0)
        {

        }

        // set; rename assign?
        void set(uint32_t width,
                 uint32_t height,
                 std::vector<Pixel> && data)
        {
            m_width = width;
            m_height = height;
            m_data = std::move(data);
        }

        void set(uint32_t width,
                 uint32_t height,
                 std::vector<Pixel> const &data)
        {
            m_width = width;
            m_height = height;
            m_data = data;
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

//        Pixel const * data()
//        {
//            return &(m_data[0]);
//        }

        std::vector<Pixel> & data()
        {
            return m_data;
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
//            uint32_t pixel_index=0;
//            for(auto &pixel : m_data) {
//                PrintPixel(pixel);
//                pixel_index++;
//                if(pixel_index%m_width==0) {
//                    std::cout << std::endl;
//                }
//            }
        }

    };

} // ilim

#endif // SCRATCH_INLINE_IMAGE_H
