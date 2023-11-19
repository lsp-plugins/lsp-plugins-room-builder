/*
 * Copyright (C) 2021 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2021 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-room-builder
 * Created on: 3 авг. 2021 г.
 *
 * lsp-plugins-room-builder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-room-builder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-room-builder. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PRIVATE_META_ROOM_BUILDER_H_
#define PRIVATE_META_ROOM_BUILDER_H_

#include <lsp-plug.in/plug-fw/meta/types.h>
#include <lsp-plug.in/plug-fw/const.h>
#include <lsp-plug.in/dsp-units/units.h>


namespace lsp
{
    namespace meta
    {
        typedef struct room_material_t
        {
            const char *name;
            const char *lc_key;
            float       speed;
            float       absorption;
        } room_material_t;

        struct room_builder_metadata
        {
            static constexpr float CONV_LENGTH_MIN          = 0.0f;     // Minimum convolution length (ms)
            static constexpr float CONV_LENGTH_MAX          = 10000.0f; // Maximum convoluition length (ms)
            static constexpr float CONV_LENGTH_DFL          = 0.0f;     // Convolution length (ms)
            static constexpr float CONV_LENGTH_STEP         = 0.1f;     // Convolution step (ms)

            static constexpr float PREDELAY_MIN             = 0.0f;     // Minimum pre-delay length (ms)
            static constexpr float PREDELAY_MAX             = 100.0f;   // Maximum pre-delay length (ms)
            static constexpr float PREDELAY_DFL             = 0.0f;     // Pre-delay length (ms)
            static constexpr float PREDELAY_STEP            = 0.01f;    // Pre-delay step (ms)

            static constexpr size_t MESH_SIZE               = 340;      // Maximum mesh size
            static constexpr size_t TRACKS_MAX              = 2;        // Maximum tracks per mesh/sample

            static constexpr size_t CONVOLVERS              = 4;        // Number of IR convolvers

            static constexpr size_t CAPTURES                = 8;        // Number of captures
            static constexpr size_t SOURCES                 = 8;        // Number of sources

            static constexpr size_t FFT_RANK_MIN            = 9;        // Minimum FFT rank

            static constexpr float POSITION_MIN             = -100.0f;
            static constexpr float POSITION_MAX             = +100.0f;
            static constexpr float POSITION_DFL             = 0.0f;
            static constexpr float POSITION_STEP            = 0.002f;

            static constexpr float OSIZE_MIN                = 10.0f;
            static constexpr float OSIZE_MAX                = 1000.0f;
            static constexpr float OSIZE_DFL                = 100.0f;
            static constexpr float OSIZE_STEP               = 0.1f;

            static constexpr float SOURCE_MIN               = 1.0f;
            static constexpr float SOURCE_MAX               = 100.0f;
            static constexpr float SOURCE_DFL               = 30.0f;
            static constexpr float SOURCE_STEP              = 0.01f;

            static constexpr float HEIGHT_MIN               = 1.0f;
            static constexpr float HEIGHT_MAX               = 100.0f;
            static constexpr float HEIGHT_DFL               = 15.0f;
            static constexpr float HEIGHT_STEP              = 0.01f;

            static constexpr float CAPSULE_MIN              = 1.0f;
            static constexpr float CAPSULE_MAX              = 30.0f;
            static constexpr float CAPSULE_DFL              = 2.2f;
            static constexpr float CAPSULE_STEP             = 0.0025f;

            static constexpr float ANGLE_MIN                = 45.0f;
            static constexpr float ANGLE_MAX                = 135.0f;
            static constexpr float ANGLE_DFL                = 90.0f;
            static constexpr float ANGLE_STEP               = 0.025f;

            static constexpr float DISTANCE_MIN             = 0.0f;
            static constexpr float DISTANCE_MAX             = +10.0f;
            static constexpr float DISTANCE_DFL             = 2.0f;
            static constexpr float DISTANCE_STEP            = 0.01f;

            static constexpr float MAT_ABSORPTION_MIN       = 0.0f;
            static constexpr float MAT_ABSORPTION_MAX       = 100.0f;
            static constexpr float MAT_ABSORPTION_DFL       = 1.0f;
            static constexpr float MAT_ABSORPTION_STEP      = 0.01f;

            static constexpr float MAT_TRANSPARENCY_MIN     = 0.0f;
            static constexpr float MAT_TRANSPARENCY_MAX     = 100.0f;
            static constexpr float MAT_TRANSPARENCY_DFL     = 50.0f;
            static constexpr float MAT_TRANSPARENCY_STEP    = 0.05f;

            static constexpr float MAT_DISPERSION_MIN       = 0.01f;
            static constexpr float MAT_DISPERSION_MAX       = 100.0f;
            static constexpr float MAT_DISPERSION_DFL       = 1.0f;
            static constexpr float MAT_DISPERSION_STEP      = 0.01f;

            static constexpr float MAT_SOUND_SPEED_MIN      = 10.0f;
            static constexpr float MAT_SOUND_SPEED_MAX      = 10000.0f;
            static constexpr float MAT_SOUND_SPEED_DFL      = LSP_DSP_UNITS_SOUND_SPEED_M_S;
            static constexpr float MAT_SOUND_SPEED_STEP     = 10.0f;

            static constexpr float LCF_MIN                  = 10.0f;
            static constexpr float LCF_MAX                  = 1000.0f;
            static constexpr float LCF_DFL                  = 50.0f;
            static constexpr float LCF_STEP                 = 0.1f;

            static constexpr float HCF_MIN                  = 2000.0f;
            static constexpr float HCF_MAX                  = 22000.0f;
            static constexpr float HCF_DFL                  = 10000.0f;
            static constexpr float HCF_STEP                 = 0.8f;

            static constexpr float BA_MIN                   = GAIN_AMP_M_12_DB;
            static constexpr float BA_MAX                   = GAIN_AMP_P_12_DB;
            static constexpr float BA_DFL                   = GAIN_AMP_0_DB;
            static constexpr float BA_STEP                  = 0.0025f;

            static constexpr float THREADS_MIN              = 1;
            static constexpr float THREADS_MAX              = 0x10000;
            static constexpr float THREADS_DFL              = 1;
            static constexpr float THREADS_STEP             = 1;

            static constexpr float DURATION_MIN             = 0.0f;
            static constexpr float DURATION_MAX             = 100000.0f;
            static constexpr float DURATION_STEP            = 1.0f;
            static constexpr float DURATION_DFL             = 0.0f;

            static constexpr size_t EQ_BANDS                = 8;        // 8 bands for equalization

            enum fft_rank_t
            {
                FFT_RANK_512,
                FFT_RANK_1024,
                FFT_RANK_2048,
                FFT_RANK_4096,
                FFT_RANK_8192,
                FFT_RANK_16384,
                FFT_RANK_32767,
                FFT_RANK_65536,

                FFT_RANK_DEFAULT = FFT_RANK_32767
            };

            static const room_material_t materials[];
        };

        extern const meta::plugin_t room_builder_mono;
        extern const meta::plugin_t room_builder_stereo;
    } // namespace meta
} // namespace lsp


#endif /* PRIVATE_META_ROOM_BUILDER_H_ */
