/*
 * Copyright (C) 2024 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2024 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#ifndef PRIVATE_PLUGINS_ROOM_BUILDER_H_
#define PRIVATE_PLUGINS_ROOM_BUILDER_H_

#include <lsp-plug.in/plug-fw/plug.h>
#include <lsp-plug.in/plug-fw/core/KVTStorage.h>
#include <lsp-plug.in/common/atomic.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/ctl/Toggle.h>
#include <lsp-plug.in/dsp-units/filters/Equalizer.h>
#include <lsp-plug.in/dsp-units/sampling/Sample.h>
#include <lsp-plug.in/dsp-units/sampling/SamplePlayer.h>
#include <lsp-plug.in/dsp-units/util/Convolver.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/3d/raytrace.h>
#include <lsp-plug.in/dsp-units/3d/Scene3D.h>
#include <lsp-plug.in/dsp-units/3d/RayTrace3D.h>
#include <lsp-plug.in/lltl/parray.h>
#include <lsp-plug.in/ipc/Thread.h>

#include <private/meta/room_builder.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Room Builder Plugin Series
         */
        class room_builder: public plug::Module
        {
            public:
                typedef struct obj_props_t
                {
                    const char             *sName;      // UTF-8 object name
                    bool                    bEnabled;   // Enabled flag
                    dsp::point3d_t          sCenter;    // Object center
                    dsp::vector3d_t         sMove;      // Object move
                    float                   fYaw;       // Yaw
                    float                   fPitch;     // Pitch
                    float                   fRoll;      // Roll
                    dsp::vector3d_t         sScale;     // Scaling of object
                    float                   fHue;       // Hue color
                    float                   fAbsorption[2];
                    float                   lnkAbsorption;
                    float                   fDispersion[2];
                    float                   lnkDispersion;
                    float                   fDiffusion[2];
                    float                   lnkDiffusion;
                    float                   fTransparency[2];
                    float                   lnkTransparency;
                    float                   fSndSpeed;
                } obj_props_t;

            protected:
                enum sync_t
                {
                    SYNC_TOGGLE_RENDER      = 1 << 0
                };

            protected:

                typedef struct convolver_t
                {
                    dspu::Delay         sDelay;         // Delay line

                    dspu::Convolver    *pCurr;          // Currently used convolver
                    dspu::Convolver    *pSwap;          // Swap

                    size_t              nSampleID;      // Sample identifier
                    size_t              nTrackID;       // Track identifier

                    float              *vBuffer;        // Buffer for convolution
                    float               fPanIn[2];      // Input panning of convolver
                    float               fPanOut[2];     // Output panning of convolver

                    plug::IPort        *pMakeup;        // Makeup gain of convolver
                    plug::IPort        *pPanIn;         // Input panning of convolver
                    plug::IPort        *pPanOut;        // Output panning of convolver
                    plug::IPort        *pSample;        // Convolver source sample
                    plug::IPort        *pTrack;         // Convolver source sample track
                    plug::IPort        *pPredelay;      // Pre-delay
                    plug::IPort        *pMute;          // Mute button
                    plug::IPort        *pActivity;      // Activity indicator
                } convolver_t;

                typedef struct channel_t
                {
                    dspu::Bypass        sBypass;
                    dspu::SamplePlayer  sPlayer;
                    dspu::Equalizer     sEqualizer;     // Wet signal equalizer

                    float              *vOut;
                    float              *vBuffer;        // Rendering buffer
                    float               fDryPan[2];     // Dry panorama

                    plug::IPort        *pOut;

                    plug::IPort        *pWetEq;         // Wet equalization flag
                    plug::IPort        *pLowCut;        // Low-cut flag
                    plug::IPort        *pLowFreq;       // Low-cut frequency
                    plug::IPort        *pHighCut;       // High-cut flag
                    plug::IPort        *pHighFreq;      // Low-cut frequency
                    plug::IPort        *pFreqGain[meta::room_builder_metadata::EQ_BANDS];    // Gain for each band of the Equalizer
                } channel_t;

                typedef struct input_t
                {
                    float                  *vIn;        // Input data
                    plug::IPort            *pIn;        // Input port
                    plug::IPort            *pPan;       // Panning
                } input_t;

                typedef struct source_t: public dspu::room_source_config_t
                {
                    bool                    bEnabled;

                    plug::IPort            *pEnabled;
                    plug::IPort            *pType;
                    plug::IPort            *pPhase;
                    plug::IPort            *pPosX;
                    plug::IPort            *pPosY;
                    plug::IPort            *pPosZ;
                    plug::IPort            *pYaw;
                    plug::IPort            *pPitch;
                    plug::IPort            *pRoll;
                    plug::IPort            *pSize;
                    plug::IPort            *pHeight;
                    plug::IPort            *pAngle;
                    plug::IPort            *pCurvature;
                } source_t;

                typedef struct capture_t: public dspu::room_capture_config_t
                {
                    dspu::Toggle            sListen;        // Listen toggle

                    bool                    bEnabled;       // Enabled flag
                    ssize_t                 nRMin;          // Minimum reflection order
                    ssize_t                 nRMax;          // Maximum reflection order

                    float                   fHeadCut;
                    float                   fTailCut;
                    float                   fFadeIn;
                    float                   fFadeOut;
                    bool                    bReverse;
                    float                   fMakeup;        // Makeup gain
                    size_t                  nLength;        // Output: length of captured response in samples
                    status_t                nStatus;        // Output: status of sample rendering
                    float                   fCurrLen;
                    float                   fMaxLen;

                    bool                    bSync;          // Sync with UI
                    bool                    bExport;        // Export flag

                    dspu::Sample           *pProcessed;     // Processed sample for playback (rendered)

                    float                  *vThumbs[meta::room_builder_metadata::TRACKS_MAX];

                    // Capture functions
                    plug::IPort            *pEnabled;
                    plug::IPort            *pRMin;
                    plug::IPort            *pRMax;
                    plug::IPort            *pPosX;
                    plug::IPort            *pPosY;
                    plug::IPort            *pPosZ;
                    plug::IPort            *pYaw;
                    plug::IPort            *pPitch;
                    plug::IPort            *pRoll;
                    plug::IPort            *pCapsule;
                    plug::IPort            *pConfig;
                    plug::IPort            *pAngle;
                    plug::IPort            *pDistance;
                    plug::IPort            *pDirection;
                    plug::IPort            *pSide;

                    // Sample editor functions
                    plug::IPort            *pHeadCut;
                    plug::IPort            *pTailCut;
                    plug::IPort            *pFadeIn;
                    plug::IPort            *pFadeOut;
                    plug::IPort            *pListen;
                    plug::IPort            *pReverse;       // Reverse
                    plug::IPort            *pMakeup;        // Makeup gain
                    plug::IPort            *pStatus;        // Status of rendering
                    plug::IPort            *pLength;        // Length of sample
                    plug::IPort            *pCurrLen;       // Current duration
                    plug::IPort            *pMaxLen;        // Max duration
                    plug::IPort            *pThumbs;        // Thumbnails of sample

                    plug::IPort            *pOutFile;       // Output file name
                    plug::IPort            *pSaveCmd;       // Save command
                    plug::IPort            *pSaveStatus;    // Save status
                    plug::IPort            *pSaveProgress;  // Save progress points
                } capture_t;

                typedef struct sample_t
                {
                    dspu::Sample                sSample;
                    size_t                      nID;
                    dspu::rt_capture_config_t   enConfig;
                } sample_t;

            protected:
                class SceneLoader: public ipc::ITask
                {
                    public:
                        size_t                  nFlags;
                        char                    sPath[PATH_MAX];
                        room_builder           *pBuilder;
                        dspu::Scene3D           sScene;

                    public:
                        inline SceneLoader()
                        {
                            nFlags          = 0;
                            sPath[0]        = '\0';
                            pBuilder        = NULL;
                        }

                        void                init(room_builder *base);
                        void                destroy();

                    public:
                        virtual status_t    run() override;
                };

                class RenderLauncher: public ipc::ITask
                {
                    public:
                        room_builder  *pBuilder;

                    public:
                        explicit inline RenderLauncher(room_builder *builder): pBuilder(builder) {}

                    public:
                        virtual status_t run() override;
                };

                class Renderer: public ipc::Thread
                {
                    protected:
                        room_builder           *pBuilder;
                        dspu::RayTrace3D       *pRT;
                        size_t                  nThreads;
                        lltl::parray<sample_t>  vSamples;
                        ipc::Mutex              lkTerminate;

                    public:
                        inline Renderer(room_builder *bld, dspu::RayTrace3D *rt, size_t threads, lltl::parray<sample_t> &samples):
                            pBuilder(bld), pRT(rt), nThreads(threads)
                        {
                            vSamples.swap(&samples);
                        }

                        virtual status_t    run() override;

                        void                terminate();
                };

                class Configurator: public ipc::ITask
                {
                    public:
                        room_builder           *pBuilder;
                        volatile uatomic_t      nChangeReq;
                        uatomic_t               nChangeResp;

                    public:
                        inline Configurator(room_builder *bld):
                            pBuilder(bld)
                        {
                            nChangeReq      = 0;
                            nChangeResp     = 0;
                        }

                        virtual status_t    run() override;

                        inline bool         need_launch() const         { return nChangeReq != nChangeResp; }

                        inline void         query_launch()              { atomic_add(&nChangeReq, 1);       }

                        inline size_t       change_req() const          { return nChangeReq;                }

                        inline void         commit(size_t change_req)   { nChangeResp = change_req;         }
                };

                class SampleSaver: public ipc::ITask
                {
                    public:
                        room_builder           *pBuilder;
                        char                    sPath[PATH_MAX+1];
                        size_t                  nSampleID;

                    public:
                        inline SampleSaver(room_builder *builder)
                        {
                            pBuilder    = builder;
                            sPath[0]    = '\0';
                            nSampleID   = 0;
                        }

                        void                bind(size_t sample_id, capture_t *capture);

                    public:
                        virtual status_t    run() override;
                };

                class GCTask: public ipc::ITask
                {
                    private:
                        room_builder           *pBuilder;

                    public:
                        explicit GCTask(room_builder *base);
                        virtual ~GCTask() override;

                    public:
                        virtual status_t run() override;

                        void        dump(dspu::IStateDumper *v) const;
                };

            protected:
                size_t                  nInputs;
                ssize_t                 nRenderThreads;
                float                   fRenderQuality;
                bool                    bRenderNormalize;
                status_t                enRenderStatus;
                float                   fRenderProgress;
                float                   fRenderCmd;
                size_t                  nFftRank;
                dspu::Sample           *pGCList;        // Garbage collection list

                input_t                 vInputs[2];
                channel_t               vChannels[2];
                convolver_t             vConvolvers[meta::room_builder_metadata::CONVOLVERS];
                capture_t               vCaptures[meta::room_builder_metadata::CAPTURES];
                source_t                vSources[meta::room_builder_metadata::SOURCES];

                dspu::Scene3D           sScene;
                dsp::vector3d_t         sScale;
                Renderer               *pRenderer;

                status_t                nSceneStatus;
                float                   fSceneProgress;
                size_t                  nSync;

                SceneLoader             s3DLoader;
                RenderLauncher          s3DLauncher;
                Configurator            sConfigurator;
                SampleSaver             sSaver;
                GCTask                  sGCTask;

                plug::IPort            *pBypass;
                plug::IPort            *pRank;
                plug::IPort            *pDry;
                plug::IPort            *pWet;
                plug::IPort            *pRenderThreads;
                plug::IPort            *pRenderQuality;
                plug::IPort            *pRenderStatus;
                plug::IPort            *pRenderProgress;
                plug::IPort            *pRenderNormalize;
                plug::IPort            *pRenderCmd;
                plug::IPort            *pOutGain;
                plug::IPort            *pPredelay;
                plug::IPort            *p3DFile;
                plug::IPort            *p3DProgress;
                plug::IPort            *p3DStatus;
                plug::IPort            *p3DOrientation;
                plug::IPort            *pScaleX;
                plug::IPort            *pScaleY;
                plug::IPort            *pScaleZ;

                void                   *pData;      // Allocated data
                ipc::IExecutor         *pExecutor;

            protected:
                static size_t       get_fft_rank(size_t rank);
                static void         destroy_convolver(dspu::Convolver * &c);
                static void         destroy_sample(dspu::Sample * &s);
                static void         destroy_samples(lltl::parray<sample_t> &samples);
                static void         destroy_gc_samples(dspu::Sample *gc_list);
                static status_t     progress_callback(float progress, void *ptr);
                static status_t     fetch_kvt_sample(core::KVTStorage *kvt, size_t sample_id, dspu::sample_header_t *hdr, const float **samples);

            protected:
                status_t            start_rendering();
                void                perform_gc();
                status_t            run_rendring(void *arg);
                status_t            bind_sources(dspu::RayTrace3D *rt);
                status_t            bind_captures(lltl::parray<sample_t> &samples, dspu::RayTrace3D *rt);
                status_t            bind_scene(core::KVTStorage *kvt, dspu::RayTrace3D *rt);
                status_t            commit_samples(lltl::parray<sample_t> &samples);
                status_t            reconfigure();
                status_t            save_sample(const char *path, size_t sample_id);

                void                process_render_requests();
                void                process_scene_load_requests();
                void                process_save_sample_requests();
                void                process_configuration_requests();
                void                process_gc_requests();
                void                process_listen_requests();
                void                perform_convolution(size_t samples);
                void                output_parameters();
                void                do_destroy();

            public:
                explicit room_builder(const meta::plugin_t *metadata, size_t inputs);
                room_builder(const room_builder &) = delete;
                room_builder(room_builder &&) = delete;
                virtual ~room_builder() override;

                room_builder & operator = (const room_builder &) = delete;
                room_builder & operator = (room_builder &&) = delete;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        update_settings() override;
                virtual void        update_sample_rate(long sr) override;

                virtual void        process(size_t samples) override;

                virtual void        state_loaded() override;
                virtual void        ui_activated() override;

            public:
                static dspu::rt_capture_config_t  decode_config(float value);
                static dspu::rt_audio_capture_t   decode_direction(float value);
                static dspu::rt_audio_capture_t   decode_side_direction(float value);
                static dspu::rt_audio_source_t    decode_source_type(float value);

                static void                 kvt_cleanup_objects(core::KVTStorage *kvt, size_t objects);
                static void                 read_object_properties(obj_props_t *props, const char *base, core::KVTStorage *kvt);
                static void                 build_object_matrix(dsp::matrix3d_t *m, const obj_props_t *props, const dsp::matrix3d_t *world);
        };
    } /* namespace plugins */
} /* namespace lsp */

#endif /* PRIVATE_PLUGINS_ROOM_BUILDER_H_ */
