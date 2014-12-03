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

// ilim
#include <ilim.hpp>

namespace ilim
{
    // ============================================================= //

    // some additional pixel types for testing

    struct G8
    {
        uint8_t g;
    };

    struct G16
    {
        uint16_t g;
    };

    struct G32
    {
        uint32_t g;
    };

    struct B8
    {
        uint8_t b;
    };

    struct B16
    {
        uint16_t b;
    };

    struct B32
    {
        uint32_t b;
    };

    struct A8
    {
        uint8_t a;
    };

    struct A16
    {
        uint16_t a;
    };

    struct A32
    {
        uint32_t a;
    };

    // ============================================================= //

    namespace ilim_detail
    {
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

        template<>
        struct pixel_traits<G16>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 16;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };

        template<>
        struct pixel_traits<G32>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 32;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<B8>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 8;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<B16>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 16;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<B32>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 32;
            static const uint8_t bits_a = 0;
        };


        template<>
        struct pixel_traits<A8>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 8;
        };


        template<>
        struct pixel_traits<A16>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 16;
        };


        template<>
        struct pixel_traits<A32>
        {
            static const uint8_t channel_count = 1;

            static const bool is_int_type = true;
            static const bool single_bitdepth = true;

            static const uint8_t bits_r = 0;
            static const uint8_t bits_g = 0;
            static const uint8_t bits_b = 0;
            static const uint8_t bits_a = 32;
        };
    } // ilim_detail
} // ilim

using namespace ilim;

//void test_channel_noop()
//{
//    std::vector<RGBA8> list_rgba8_src(1,{11,22,33,44});
//    list_r8_dst.clear();
//    std::vector<G8> list_g8_dst;
//    std::vector<B8> list_b8_dst;
//    std::vector<A8> list_a8_dst;

//    ilim_detail::conv_pixels(list_rgba8_src,list_r8_dst);
//    assert(list_r8_dst[0].r == 11);

//    ilim_detail::conv_pixels(list_rgba8_src,list_g8_dst);
//    assert(list_g8_dst[0].g == 22);

//    ilim_detail::conv_pixels(list_rgba8_src,list_b8_dst);
//    assert(list_b8_dst[0].b == 33);

//    ilim_detail::conv_pixels(list_rgba8_src,list_a8_dst);
//    assert(list_a8_dst[0].a == 44);
//}

void test_channel_noop()
{
    // no op is called on the assign when the
    // destination channel doesn't exist

    // cant explicitly check outside of ensuring
    // conv_pixels with types that require no ops
    // between channels compile/don't have an error

    std::vector<RGBA8> list_rgba8_src(1,{11,22,33,44});
    std::vector<R8> list_r8_dst;
    std::vector<G8> list_g8_dst;
    std::vector<B8> list_b8_dst;
    std::vector<A8> list_a8_dst;

    ilim_detail::conv_pixels(list_rgba8_src,list_r8_dst);
    assert(list_r8_dst[0].r == 11);

    ilim_detail::conv_pixels(list_rgba8_src,list_g8_dst);
    assert(list_g8_dst[0].g == 22);

    ilim_detail::conv_pixels(list_rgba8_src,list_b8_dst);
    assert(list_b8_dst[0].b == 33);

    ilim_detail::conv_pixels(list_rgba8_src,list_a8_dst);
    assert(list_a8_dst[0].a == 44);

    std::cout << "test_channel_noop... [ok]" << std::endl;
}

void test_channel_sub()
{
    // G8 -> R8
    // expect r8 value to be subbed in to 0
    std::vector<G8> list_g8_src(1,{99});
    std::vector<R8> list_r8_dst;
    ilim_detail::conv_pixels(list_g8_src,list_r8_dst);
    assert(list_r8_dst[0].r == 0);

    // B8 -> A8
    // expect a8 value to be subbed in to 1 (alpha defaults to 1)
    std::vector<B8> list_b8_src(1,{99});
    std::vector<A8> list_a8_dst;
    ilim_detail::conv_pixels(list_b8_src,list_a8_dst);
    assert(list_a8_dst[0].a == 1);

    std::cout << "test_channel_sub... [ok]" << std::endl;
}

void test_channel_same_bitdepth()
{
    // red
    {
        std::vector<R8> list_r8_src(10,{99});
        std::vector<R8> list_r8_dst;
        ilim_detail::conv_pixels(list_r8_src,list_r8_dst);
        assert(list_r8_dst[0].r == 99);

        std::vector<R16> list_r16_src(10,{32000});
        std::vector<R16> list_r16_dst;
        ilim_detail::conv_pixels(list_r16_src,list_r16_dst);
        assert(list_r16_dst[0].r == 32000);

        std::vector<R32> list_r32_src(10,{100000});
        std::vector<R32> list_r32_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r32_dst);
        assert(list_r32_dst[0].r == 100000);
    }

    // green
    {
        std::vector<G16> list_16_src(10,{32000});
        std::vector<G16> list_16_dst;
        ilim_detail::conv_pixels(list_16_src,list_16_dst);
        assert(list_16_dst[0].g == 32000);

        std::vector<G32> list_32_src(10,{100000});
        std::vector<G32> list_32_dst;
        ilim_detail::conv_pixels(list_32_src,list_32_dst);
        assert(list_32_dst[0].g == 100000);
    }

    // blue
    {
        std::vector<B8> list_8_src(10,{99});
        std::vector<B8> list_8_dst;
        ilim_detail::conv_pixels(list_8_src,list_8_dst);
        assert(list_8_dst[0].b == 99);

        std::vector<B16> list_16_src(10,{32000});
        std::vector<B16> list_16_dst;
        ilim_detail::conv_pixels(list_16_src,list_16_dst);
        assert(list_16_dst[0].b == 32000);

        std::vector<B32> list_32_src(10,{100000});
        std::vector<B32> list_32_dst;
        ilim_detail::conv_pixels(list_32_src,list_32_dst);
        assert(list_32_dst[0].b == 100000);
    }

    // alpha
    {
        std::vector<A8> list_8_src(10,{99});
        std::vector<A8> list_8_dst;
        ilim_detail::conv_pixels(list_8_src,list_8_dst);
        assert(list_8_dst[0].a == 99);

        std::vector<A16> list_16_src(10,{32000});
        std::vector<A16> list_16_dst;
        ilim_detail::conv_pixels(list_16_src,list_16_dst);
        assert(list_16_dst[0].a == 32000);

        std::vector<A32> list_32_src(10,{100000});
        std::vector<A32> list_32_dst;
        ilim_detail::conv_pixels(list_32_src,list_32_dst);
        assert(list_32_dst[0].a == 100000);
    }

    std::cout << "test_channel_same_bitdepth... [ok]" << std::endl;
}

void test_channel_downsample()
{
    // red
    {
        std::vector<R32> list_r32_src = {
            {0},
            {ilim_detail::ct_ui_pow(2,4)},
            {ilim_detail::ct_ui_pow(2,8)},
            {ilim_detail::ct_ui_pow(2,12)},
            {ilim_detail::ct_ui_pow(2,16)},
            {ilim_detail::ct_ui_pow(2,20)},
            {ilim_detail::ct_ui_pow(2,24)},
            {ilim_detail::ct_ui_pow(2,28)},
            {ilim_detail::ct_ui_pow(2,32)-1}};

        std::vector<R16> list_r16_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r16_dst);

        std::vector<R8> list_r8_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r8_dst);

        for(size_t i=0; i < list_r32_src.size(); i++) {
            assert(list_r32_src[i].r/(ilim_detail::ct_ui_pow(2,16)) == list_r16_dst[i].r);
            assert(list_r32_src[i].r/(ilim_detail::ct_ui_pow(2,24)) == list_r8_dst[i].r);
        }
    }

    // green
    {
        std::vector<G32> list_r32_src = {
            {0},
            {ilim_detail::ct_ui_pow(2,4)},
            {ilim_detail::ct_ui_pow(2,8)},
            {ilim_detail::ct_ui_pow(2,12)},
            {ilim_detail::ct_ui_pow(2,16)},
            {ilim_detail::ct_ui_pow(2,20)},
            {ilim_detail::ct_ui_pow(2,24)},
            {ilim_detail::ct_ui_pow(2,28)},
            {ilim_detail::ct_ui_pow(2,32)-1}};

        std::vector<G16> list_r16_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r16_dst);

        std::vector<G8> list_r8_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r8_dst);

        for(size_t i=0; i < list_r32_src.size(); i++) {
            assert(list_r32_src[i].g/(ilim_detail::ct_ui_pow(2,16)) == list_r16_dst[i].g);
            assert(list_r32_src[i].g/(ilim_detail::ct_ui_pow(2,24)) == list_r8_dst[i].g);
        }
    }

    // blue
    {
        std::vector<B32> list_r32_src = {
            {0},
            {ilim_detail::ct_ui_pow(2,4)},
            {ilim_detail::ct_ui_pow(2,8)},
            {ilim_detail::ct_ui_pow(2,12)},
            {ilim_detail::ct_ui_pow(2,16)},
            {ilim_detail::ct_ui_pow(2,20)},
            {ilim_detail::ct_ui_pow(2,24)},
            {ilim_detail::ct_ui_pow(2,28)},
            {ilim_detail::ct_ui_pow(2,32)-1}};

        std::vector<B16> list_r16_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r16_dst);

        std::vector<B8> list_r8_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r8_dst);

        for(size_t i=0; i < list_r32_src.size(); i++) {
            assert(list_r32_src[i].b/(ilim_detail::ct_ui_pow(2,16)) == list_r16_dst[i].b);
            assert(list_r32_src[i].b/(ilim_detail::ct_ui_pow(2,24)) == list_r8_dst[i].b);
        }
    }

    // alpha
    {
        std::vector<A32> list_r32_src = {
            {0},
            {ilim_detail::ct_ui_pow(2,4)},
            {ilim_detail::ct_ui_pow(2,8)},
            {ilim_detail::ct_ui_pow(2,12)},
            {ilim_detail::ct_ui_pow(2,16)},
            {ilim_detail::ct_ui_pow(2,20)},
            {ilim_detail::ct_ui_pow(2,24)},
            {ilim_detail::ct_ui_pow(2,28)},
            {ilim_detail::ct_ui_pow(2,32)-1}};

        std::vector<A16> list_r16_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r16_dst);

        std::vector<A8> list_r8_dst;
        ilim_detail::conv_pixels(list_r32_src,list_r8_dst);

        for(size_t i=0; i < list_r32_src.size(); i++) {
            assert(list_r32_src[i].a/(ilim_detail::ct_ui_pow(2,16)) == list_r16_dst[i].a);
            assert(list_r32_src[i].a/(ilim_detail::ct_ui_pow(2,24)) == list_r8_dst[i].a);
        }
    }

    std::cout << "test_channel_downsample... [ok]" << std::endl;
}

void test_channel_upsample()
{
    // red
    {
        std::vector<R8> list_8_src = {{0},{2},{4},{8},{16},{32},{64},{128},{255}};

        std::vector<R16> list_16_dst;
        ilim_detail::conv_pixels(list_8_src,list_16_dst);

        std::vector<R32> list_32_dst;
        ilim_detail::conv_pixels(list_8_src,list_32_dst);

        for(size_t i=0; i < list_8_src.size(); i++) {
            assert(list_16_dst[i].r/256 == list_8_src[i].r);
            assert(list_32_dst[i].r/uint64_t(16777216) == list_8_src[i].r); // 16777216 == 2^24
        }
    }

    // green
    {
        std::vector<G8> list_8_src = {{0},{2},{4},{8},{16},{32},{64},{128},{255}};

        std::vector<G16> list_16_dst;
        ilim_detail::conv_pixels(list_8_src,list_16_dst);

        std::vector<G32> list_32_dst;
        ilim_detail::conv_pixels(list_8_src,list_32_dst);

        for(size_t i=0; i < list_8_src.size(); i++) {
            assert(list_16_dst[i].g/256 == list_8_src[i].g);
            assert(list_32_dst[i].g/uint64_t(16777216) == list_8_src[i].g); // 16777216 == 2^24
        }
    }

    // blue
    {
        std::vector<B8> list_8_src = {{0},{2},{4},{8},{16},{32},{64},{128},{255}};

        std::vector<B16> list_16_dst;
        ilim_detail::conv_pixels(list_8_src,list_16_dst);

        std::vector<B32> list_32_dst;
        ilim_detail::conv_pixels(list_8_src,list_32_dst);

        for(size_t i=0; i < list_8_src.size(); i++) {
            assert(list_16_dst[i].b/256 == list_8_src[i].b);
            assert(list_32_dst[i].b/uint64_t(16777216) == list_8_src[i].b); // 16777216 == 2^24
        }
    }

    // alpha
    {
        std::vector<A8> list_8_src = {{0},{2},{4},{8},{16},{32},{64},{128},{255}};

        std::vector<A16> list_16_dst;
        ilim_detail::conv_pixels(list_8_src,list_16_dst);

        std::vector<A32> list_32_dst;
        ilim_detail::conv_pixels(list_8_src,list_32_dst);

        for(size_t i=0; i < list_8_src.size(); i++) {
            assert(list_16_dst[i].a/256 == list_8_src[i].a);
            assert(list_32_dst[i].a/uint64_t(16777216) == list_8_src[i].a); // 16777216 == 2^24
        }
    }

    std::cout << "test_channel_upsample... [ok]" << std::endl;
}

int main()
{
    test_channel_noop();
    test_channel_sub();
    test_channel_same_bitdepth();
    test_channel_downsample();
    test_channel_upsample();

    return 0;
}
