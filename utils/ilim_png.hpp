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

#ifndef SCRATCH_INLINE_IMAGE_PNG_H
#define SCRATCH_INLINE_IMAGE_PNG_H

#include <lodepng/lodepng.h>
#include <ilim.hpp>

namespace ilim
{
    // enum values are for the equivalent value in
    // PNG header data
    enum class PNGColorType {
        GREY        = 0, /*greyscale: 1,2,4,8,16 bit*/
        RGB         = 2, /*RGB: 8,16 bit*/
        PALETTE     = 3, /*palette: 1,2,4,8 bit*/
        GREY_ALPHA  = 4, /*greyscale with alpha: 8,16 bit*/
        RGBA        = 6  /*RGB with alpha: 8,16 bit*/
    };

    inline bool load_png_format(std::vector<uint8_t> const &png_data,
                                PNGColorType &color_type,
                                uint8_t &bit_depth)
    {
        if(png_data.empty() || png_data.size() < 25) {
            std::cout << "ERROR: Image: Failed to load png"
                         << ": Invalid header" << std::endl;
            return false;
        }

        // Check the color type and bit depth

        // PNG header:
        // * 8 bytes contain a fixed png id
        // * 4 bytes contain chunk length for the IHDR chunk
        // * 4 bytes contain chunk name for the IHDR chunk
        // * 4 bytes contain width
        // * 4 bytes contain height
        // * 1 byte bit depth  <-- value we want
        // * 1 byte color type <-- value we want
        // ...
        // Multi-byte values are big endian

        bit_depth = png_data[24];
        color_type = static_cast<PNGColorType>(png_data[25]);
        return true;
    }

    inline bool load_png_format(std::string const &filepath,
                                PNGColorType &color_type,
                                uint8_t &bitdepth)
    {
        // Load the file into memory
        std::vector<uint8_t> png_data;
        lodepng::load_file(png_data,filepath);
        return load_png_format(png_data,color_type,bitdepth);
    }

    template <typename Pixel>
    bool load_png(std::vector<uint8_t> const &png_data,
                  Image<Pixel> &image)
    {
        // If the source png data colortype and bitdepth
        // match the requested format, lodepng will provide
        // the decoded data in that format

        // Otherwise, lodepng will only convert either to
        // RGB8 or RGBA8 (see lodepng.cpp, lines 4610 and on)

        // Default load opts:
        lodepng::State state;
        state.info_raw.bitdepth = 8;
        state.info_raw.colortype = LCT_RGBA;
        bool default_convert = true;

        // Get the source format
        PNGColorType src_colortype = static_cast<PNGColorType>(png_data[25]);
        uint8_t const src_bitdepth = png_data[24];
        uint8_t const src_channels =
                (src_colortype == PNGColorType::GREY) ? 1 :
                (src_colortype == PNGColorType::GREY_ALPHA) ? 2 :
                (src_colortype == PNGColorType::RGB) ? 3 :
                (src_colortype == PNGColorType::RGBA) ? 4 : 0;

        // Get the requested format
        uint8_t const req_bitdepth = ilim_detail::is_single_bitdepth_int_type<Pixel>();
        uint8_t const req_channels = ilim_detail::pixel_traits<Pixel>::channel_count;

        if((src_bitdepth == req_bitdepth) &&
           (src_channels == req_channels))
        {
            // lodepng should decode into the requested
            // format since it matches the png data
            state.info_raw.bitdepth = src_bitdepth;
            state.info_raw.colortype = static_cast<LodePNGColorType>(src_colortype);
            default_convert = false;
        }

        // Decode the png
        unsigned width,height;
        std::vector<uint8_t> list_bytes;
        unsigned error = lodepng::decode(list_bytes,width,height,state,png_data);

        if(error) {
            std::cout << "ERROR: Image: Failed to load png "
                      << ": " << lodepng_error_text(error) << std::endl;
            return false;
        }
        else {
            std::vector<Pixel> list_pixels;

            if(!default_convert) {
                list_pixels.reserve(width*height);

                // A GREY color type supports bitdepths of 1,2,4,8,16
                if(src_channels == 1) {
                    if(src_bitdepth == 1) {
                        for(uint8_t const byte : list_bytes) {
                            list_pixels.emplace_back(Pixel{ byte & 1 });
                            list_pixels.emplace_back(Pixel{ byte & 2 });
                            list_pixels.emplace_back(Pixel{ byte & 4 });
                            list_pixels.emplace_back(Pixel{ byte & 8 });
                            list_pixels.emplace_back(Pixel{ byte & 16 });
                            list_pixels.emplace_back(Pixel{ byte & 32 });
                            list_pixels.emplace_back(Pixel{ byte & 64 });
                            list_pixels.emplace_back(Pixel{ byte & 128 });
                        }
                    }
                    else if(src_bitdepth == 2) {
                        for(uint8_t const byte : list_bytes) {
                            list_pixels.emplace_back(Pixel{ byte & 3 });
                            list_pixels.emplace_back(Pixel{ byte & 12 });
                            list_pixels.emplace_back(Pixel{ byte & 48 });
                            list_pixels.emplace_back(Pixel{ byte & 192 });
                        }
                    }
                    else if(src_bitdepth == 4) {
                        for(uint8_t const byte : list_bytes) {
                            list_pixels.emplace_back(Pixel{ byte & 15 });
                            list_pixels.emplace_back(Pixel{ byte & 240 });
                        }
                    }
                    else if(src_bitdepth == 8) {
                        for(uint8_t const byte : list_bytes) {
                            list_pixels.emplace_back(Pixel{ byte });
                        }
                    }
                    else if(src_bitdepth == 16) {
                        // png data is in big endian
                        for(size_t i=0; i < list_bytes.size(); i+=2) {
                            // TODO check platform endianess! (currently LE only)
                            list_pixels.emplace_back(Pixel {
                                (static_cast<uint16_t>(list_bytes[i+1]) << 8) | list_bytes[i]
                            });
                        }
                    }
                }

                // All other types (GREY_ALPHA,RGB,RGBA) support 8,16

                // GREY_ALPHA
                else if(src_channels == 2) {
                    if(src_bitdepth == 8) {
                        for(size_t i=0; i < list_bytes.size(); i+=2) {
                            list_pixels.emplace_back(Pixel {
                                list_bytes[i],
                                list_bytes[i+1]
                            });
                        }
                    }
                    else { // src_bitdepth == 16
                        for(size_t i=0; i < list_bytes.size(); i+=4) {
                            // TODO check platform endianess! (currently LE only)
                            list_pixels.emplace_back(Pixel {
                                ((static_cast<uint16_t>(list_bytes[i+1]) << 8) | list_bytes[i+0]),
                                ((static_cast<uint16_t>(list_bytes[i+3]) << 8) | list_bytes[i+2])
                            });
                        }
                    }
                }
                // RGB
                else if(src_channels == 3) {
                    if(src_bitdepth == 8) {
                        for(size_t i=0; i < list_bytes.size(); i+=3) {
                            list_pixels.emplace_back(Pixel {
                                list_bytes[i+0],
                                list_bytes[i+1],
                                list_bytes[i+2]
                            });
                        }
                    }
                    else { // src_bitdepth == 16
                        for(size_t i=0; i < list_bytes.size(); i+=6) {
                            // TODO check platform endianess! (currently LE only)
                            list_pixels.emplace_back(Pixel {
                                ((static_cast<uint16_t>(list_bytes[i+1]) << 8) | list_bytes[i+0]),
                                ((static_cast<uint16_t>(list_bytes[i+3]) << 8) | list_bytes[i+2]),
                                ((static_cast<uint16_t>(list_bytes[i+5]) << 8) | list_bytes[i+4])
                            });
                        }
                    }
                }
                // RGBA
                else if(src_channels == 4) {
                    if(src_bitdepth == 8) {
                        for(size_t i=0; i < list_bytes.size(); i+=4) {
                            list_pixels.emplace_back(Pixel {
                                list_bytes[i+0],
                                list_bytes[i+1],
                                list_bytes[i+2],
                                list_bytes[i+3]
                            });
                        }
                    }
                    else { // src_bitdepth == 16
                        for(size_t i=0; i < list_bytes.size(); i+=8) {
                            // TODO check platform endianess! (currently LE only)
                            list_pixels.emplace_back(Pixel {
                                ((static_cast<uint16_t>(list_bytes[i+1]) << 8) | list_bytes[i+0]),
                                ((static_cast<uint16_t>(list_bytes[i+3]) << 8) | list_bytes[i+2]),
                                ((static_cast<uint16_t>(list_bytes[i+5]) << 8) | list_bytes[i+4]),
                                ((static_cast<uint16_t>(list_bytes[i+7]) << 8) | list_bytes[i+6])
                            });
                        }
                    }
                }
            }
            else {
                // The source colortype and bitdepth didn't match
                // the corresponding requested params, so we request
                // the data in RGBA8 format and conver to the desired
                // target <Pixel> type
                std::vector<RGBA8> list_temp_pixels;
                list_temp_pixels.reserve(width*height);

                for(size_t i=0; i < list_bytes.size(); i+=4) {
                    list_temp_pixels.emplace_back(Pixel {
                        list_bytes[i+0],
                        list_bytes[i+1],
                        list_bytes[i+2],
                        list_bytes[i+3]
                    });
                }

                ilim_detail::conv_pixels(list_temp_pixels,list_pixels);
            }

            // Save data to image
            image.set(width,height,std::move(list_pixels));

        }
        return true;
    }

} // ilim

#endif // SCRATCH_INLINE_IMAGE_PNG_H
