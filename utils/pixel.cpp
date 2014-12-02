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

// sys
#include <cstdint>

// stl
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <cassert>
#include <limits>

// lodepng
//#include <lodepng/lodepng.h>

namespace scratch
{  
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

    // compile-time int pow // VERIFY its compile time
    constexpr uint64_t ct_ui_pow(uint64_t base, uint64_t exp)
    {
        return ((exp == 0) ? 1 : base*ct_ui_pow(base,exp-1));
    }


    // do nothing (src exists, dst does not)
    template<typename PixelSrc,typename PixelDst>
    void upscale_r(PixelSrc const &src, PixelDst &dst, decltype(PixelDst::r) mul, decltype(PixelSrc::r) div)
    {
        dst.r = (mul/div)*src.r;
    }

    template<typename PixelSrc,typename PixelDst>
    void downscale_r(PixelSrc const &src, PixelDst &dst, uint8_t shift_count)
    {
        dst.r = (src.r >> shift_count);
    }

    template<typename PixelDst>
    void sub_r(PixelDst &dst)
    {
        dst.r = 0;
    }

    void no_op()
    {

    }

    template<typename PixelSrc,typename PixelDst>
    void assign_r(PixelSrc const &src, PixelDst &dst)
    {
        typedef pixel_traits<PixelSrc> src_traits;
        typedef pixel_traits<PixelDst> dst_traits;

        // src channel exists, dst channel does not
        // NOTE/TODO: cant ever get here for 'r'
        constexpr bool noop = (dst_traits::bits_r == 0);

        // src channel dne, dst channel exists
        constexpr bool sub = (src_traits::bits_r == 0);

        // upscale if the dst channel has
        // more bits than the src channel
        constexpr bool upscale = (dst_traits::bits_r > src_traits::bits_r);

        // the number of bits to shift the src channel
        // by when downscaling
        constexpr uint8_t ds_shift = src_traits::bits_r-dst_traits::bits_r;

        constexpr decltype(PixelSrc::r) us_div = ct_ui_pow(2,src_traits::bits_r)-1;
        constexpr decltype(PixelDst::r) us_mul = ct_ui_pow(2,dst_traits::bits_r)-1;

        (noop) ? no_op() : (sub ? sub_r(dst) : (upscale ? upscale_r(src,dst,us_mul,us_div) : downscale_r(src,dst,ds_shift)));
    }

    template <typename PixelSrc,typename PixelDst>
    void conv_pixels(std::vector<PixelSrc> const &list_src,
                     std::vector<PixelDst> &list_dst)
    {
        typedef pixel_traits<PixelSrc> src_traits;
        typedef pixel_traits<PixelDst> dst_traits;

        list_dst.clear();
        list_dst.reserve(list_src.size());

        if(src_traits::is_int_type) {
            if(dst_traits::is_int_type) {
                // int -> int
                for(auto const &src : list_src) {
                    PixelDst dst;
                    assign_r(src,dst);
                    //assign_g(src,dst);
                    //assign_b(src,dst);
                    //assign_a(src,dst);

                    list_dst.push_back(dst);
                }
            }
        }
    }

    // ============================================================= //

    struct R8
    {
        uint8_t r;
    };

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

    // ============================================================= //

    struct R16
    {
        uint16_t r;
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

    // ============================================================= //

    struct R32
    {
        uint32_t r;
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

    // ============================================================= //

    struct G8
    {
        uint8_t g;
    };

    template<>
    struct pixel_traits<G8>
    {
        static const uint8_t channel_count = 1;

        static const bool is_int_type = true;
        static const bool single_bitdepth = true;

        static const uint8_t bits_r = 0;
        static const uint8_t bits_g = 8;
        static const uint8_t bits_b = 0;
        static const uint8_t bits_a = 0;
    };

    // ============================================================= //

    struct RGB555 // TODO check memory layout
    {
        uint8_t r : 5;
        uint8_t g : 5;
        uint8_t b : 5;
        uint8_t : 1; // fill to 16 bits
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

    // ============================================================= //

    struct RGB565 // TODO check memory layout
    {
        uint8_t r : 5;
        uint8_t g : 6;
        uint8_t b : 5;
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

    // ============================================================= //

    struct RGB8
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
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

    // ============================================================= //

    struct RGBA8
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
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

    // ============================================================= //

    struct RGB32F
    {
        float r;
        float g;
        float b;
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

    // ============================================================= //

    struct RGBA32F
    {
        float r;
        float g;
        float b;
        float a;
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

    // ============================================================= //

    struct RGBA64F
    {
        double r;
        double g;
        double b;
        double a;
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
}


using namespace scratch;


void test_r_same_channel_depth()
{
    std::vector<R8> list_r8_src(10,{99});
    std::vector<R8> list_r8_dst;
    conv_pixels(list_r8_src,list_r8_dst);
    assert(list_r8_dst[0].r == 99);

    std::vector<R16> list_r16_src(10,{32000});
    std::vector<R16> list_r16_dst;
    conv_pixels(list_r16_src,list_r16_dst);
    assert(list_r16_dst[0].r == 32000);

    std::vector<R32> list_r32_src(10,{100000});
    std::vector<R32> list_r32_dst;
    conv_pixels(list_r32_src,list_r32_dst);
    assert(list_r32_dst[0].r == 100000);
}

void test_r_decrease_channel_depth()
{
    std::vector<R32> list_r32_src = {
        {0},
        {ct_ui_pow(2,4)},
        {ct_ui_pow(2,8)},
        {ct_ui_pow(2,12)},
        {ct_ui_pow(2,16)},
        {ct_ui_pow(2,20)},
        {ct_ui_pow(2,24)},
        {ct_ui_pow(2,28)},
        {ct_ui_pow(2,32)-1}};

    std::vector<R16> list_r16_dst;
    conv_pixels(list_r32_src,list_r16_dst);

    std::vector<R8> list_r8_dst;
    conv_pixels(list_r32_src,list_r8_dst);

    for(size_t i=0; i < list_r32_src.size(); i++) {
        assert(list_r32_src[i].r/(ct_ui_pow(2,16)) == list_r16_dst[i].r);
        assert(list_r32_src[i].r/(ct_ui_pow(2,24)) == list_r8_dst[i].r);
    }
}

void test_r_increase_channel_depth()
{
    std::vector<R8> list_r8_src = {{0},{2},{4},{8},{16},{32},{64},{128},{255}};

    std::vector<R16> list_r16_dst;
    conv_pixels(list_r8_src,list_r16_dst);

    std::vector<R32> list_r32_dst;
    conv_pixels(list_r8_src,list_r32_dst);

    for(size_t i=0; i < list_r8_src.size(); i++) {
        assert(list_r16_dst[i].r/256 == list_r8_src[i].r);
        assert(list_r32_dst[i].r/uint64_t(16777216) == list_r8_src[i].r); // 16777216 == 2^24
    }
}

int main()
{
    test_r_same_channel_depth();
    test_r_decrease_channel_depth();
    test_r_increase_channel_depth();

    return 0;
}
