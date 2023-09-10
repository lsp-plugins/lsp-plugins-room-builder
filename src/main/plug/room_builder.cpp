/*
 * Copyright (C) 2023 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2023 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/common/endian.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/dsp-units/misc/fade.h>
#include <lsp-plug.in/fmt/lspc/lspc.h>
#include <lsp-plug.in/fmt/lspc/AudioWriter.h>
#include <lsp-plug.in/stdlib/stdio.h>

#include <private/plugins/room_builder.h>

#define TMP_BUF_SIZE            4096
#define CONV_RANK               10
#define TRACE_PORT(p)           lsp_trace("  port id=%s", (p)->metadata()->id);

namespace lsp
{
    namespace plugins
    {
        static const float band_freqs[] =
        {
            73.0f,
            156.0f,
            332.0f,
            707.0f,
            1507.0f,
            3213.0f,
            6849.0f
        };

        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::room_builder_mono,
            &meta::room_builder_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new room_builder(meta, (meta == &meta::room_builder_stereo) ? 2 : 1);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //-------------------------------------------------------------------------
        template <class T>
            static bool kvt_fetch(core::KVTStorage *s, const char *base, const char *branch, T *value, T dfl)
            {
                char name[0x100]; // Should be enough;
                size_t len = ::strlen(base) + ::strlen(branch) + 2;
                if (len >= 0x100)
                    return false;

                char *tail = ::stpcpy(name, base);
                *(tail++)  = '/';
                stpcpy(tail, branch);

                return s->get_dfl(name, value, dfl);
            }

        template <class T>
            static bool kvt_deploy(core::KVTStorage *s, const char *base, const char *branch, T value, size_t flags)
            {
                char name[0x100]; // Should be enough
                size_t len = ::strlen(base) + ::strlen(branch) + 2;
                if (len >= 0x100)
                    return false;

                char *tail = ::stpcpy(name, base);
                *(tail++)  = '/';
                stpcpy(tail, branch);

                return s->put(name, value, flags) == STATUS_OK;
            }

        //-------------------------------------------------------------------------
        // 3D Scene loader
        void room_builder::SceneLoader::init(room_builder *base)
        {
            pBuilder   = base;
            sScene.clear();
        }

        void room_builder::SceneLoader::destroy()
        {
            sScene.destroy();
        }

        status_t room_builder::SceneLoader::run()
        {
            // Clear scene
            sScene.clear();

            // Check state
            size_t nobjs = 0;
            status_t res = STATUS_UNSPECIFIED;

            // Load the scene file
            if (pBuilder->p3DFile == NULL)
                res = STATUS_UNKNOWN_ERR;
            else if (::strlen(sPath) > 0)
            {
                lsp_trace("Loading scene from %s", sPath);

                // Load file from resources
                io::IInStream *is = pBuilder->pWrapper->resources()->read_stream(sPath);
                if (is == NULL)
                    return pBuilder->pWrapper->resources()->last_error();

                res = sScene.load(is);
                status_t res2 = is->close();
                delete is;

                res = (res == STATUS_OK) ? res2 : res;

                if (res == STATUS_OK)
                {
                    // Initialize object properties
                    nobjs = sScene.num_objects();
                }
            }
            else
                lsp_trace("Scene file name not specified");

            // Get KVT storage and deploy new values
            core::KVTStorage *kvt = pBuilder->kvt_lock();
            if (kvt == NULL)
                return STATUS_UNKNOWN_ERR;

            // Now initialize object properties
            lsp_trace("Extra loading flags=0x%x", int(nFlags));
            size_t f_extra  = (nFlags & (plug::PF_STATE_IMPORT | plug::PF_PRESET_IMPORT | plug::PF_STATE_RESTORE)) ?
                                core::KVT_KEEP | core::KVT_TX : core::KVT_TX;
            size_t f_hue    = (nFlags & (plug::PF_STATE_IMPORT | plug::PF_STATE_RESTORE)) ?
                                core::KVT_KEEP | core::KVT_TX : core::KVT_TX;

            char base[128];
            kvt_deploy(kvt, "/scene", "objects", int32_t(nobjs), core::KVT_TX);
            kvt_deploy(kvt, "/scene", "selected", 0.0f, f_extra);

            for (size_t i=0; i<nobjs; ++i)
            {
                dspu::Object3D *obj = sScene.object(i);
                if (obj == NULL)
                {
                    res = STATUS_UNKNOWN_ERR;
                    break;
                }
                const dsp::point3d_t *c  = obj->center();

                sprintf(base, "/scene/object/%d", int(i));
                lsp_trace("Deploying KVT parameters for %s", base);

                kvt_deploy(kvt, base, "name", obj->get_name(), core::KVT_TX); // Always overwrite name

                kvt_deploy(kvt, base, "enabled", 1.0f, f_extra);
                kvt_deploy(kvt, base, "center/x", c->x, core::KVT_TX | core::KVT_TRANSIENT); // Always overwrite, do not save in state
                kvt_deploy(kvt, base, "center/y", c->y, core::KVT_TX | core::KVT_TRANSIENT); // Always overwrite, do not save in state
                kvt_deploy(kvt, base, "center/z", c->z, core::KVT_TX | core::KVT_TRANSIENT); // Always overwrite, do not save in state
                kvt_deploy(kvt, base, "position/x", 0.0f, f_extra);
                kvt_deploy(kvt, base, "position/y", 0.0f, f_extra);
                kvt_deploy(kvt, base, "position/z", 0.0f, f_extra);
                kvt_deploy(kvt, base, "rotation/yaw", 0.0f, f_extra);
                kvt_deploy(kvt, base, "rotation/pitch", 0.0f, f_extra);
                kvt_deploy(kvt, base, "rotation/roll", 0.0f, f_extra);
                kvt_deploy(kvt, base, "scale/x", 100.0f, f_extra);
                kvt_deploy(kvt, base, "scale/y", 100.0f, f_extra);
                kvt_deploy(kvt, base, "scale/z", 100.0f, f_extra);
                kvt_deploy(kvt, base, "color/hue", float(i) / float(nobjs), f_hue); // Always overwrite hue

                kvt_deploy(kvt, base, "material/absorption/outer", 1.5f, f_extra); // Absorption of concrete material
                kvt_deploy(kvt, base, "material/dispersion/outer", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/diffusion/outer", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/transparency/outer", 48.0f, f_extra);

                kvt_deploy(kvt, base, "material/absorption/inner", 1.5f, f_extra);
                kvt_deploy(kvt, base, "material/dispersion/inner", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/diffusion/inner", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/transparency/inner", 52.0f, f_extra);

                kvt_deploy(kvt, base, "material/absorption/link", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/dispersion/link", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/diffusion/link", 1.0f, f_extra);
                kvt_deploy(kvt, base, "material/transparency/link", 1.0f, f_extra);

                kvt_deploy(kvt, base, "material/sound_speed", 4250.0f, f_extra);  // Sound speed in concrete material
            }

            // Drop rare (unused) objects
            kvt_cleanup_objects(kvt, nobjs);

            pBuilder->kvt_release();

            return res;
        }

        //-------------------------------------------------------------------------
        status_t room_builder::RenderLauncher::run()
        {
            return pBuilder->start_rendering();
        }

        //-------------------------------------------------------------------------
        status_t room_builder::Renderer::run()
        {
            // Perform processing
            lsp_trace("Launching process() method");
            pBuilder->enRenderStatus    = STATUS_IN_PROCESS;
            status_t res    = pRT->process(nThreads, 1.0f);

            // Deploy success result
            if (res == STATUS_OK)
                res = pBuilder->commit_samples(vSamples);

            // Free all resources
            if (lkTerminate.lock())
            {
                pRT->destroy(true);
                delete pRT;
                pRT = NULL;
                lkTerminate.unlock();
            }

            room_builder::destroy_samples(vSamples);

            return pBuilder->enRenderStatus = res;
        }

        void room_builder::Renderer::terminate()
        {
            if (lkTerminate.lock())
            {
                if (pRT != NULL)
                    pRT->cancel();
                lkTerminate.unlock();
            }
        }

        //-------------------------------------------------------------------------
        status_t room_builder::Configurator::run()
        {
            return pBuilder->reconfigure();
        }

        //-------------------------------------------------------------------------
        void room_builder::SampleSaver::bind(size_t sample_id, capture_t *capture)
        {
            nSampleID       = sample_id;
            plug::IPort *p  = capture->pOutFile;
            if (p == NULL)
                return;
            plug::path_t *path = p->buffer<plug::path_t>();
            if (path == NULL)
                return;
            const char *spath = path->path();
            if (spath != NULL)
            {
                ::strncpy(sPath, spath, PATH_MAX);
                sPath[PATH_MAX] = '\0';
            }
            else
                sPath[0] = '\0';
        }

        status_t room_builder::SampleSaver::run()
        {
            return pBuilder->save_sample(sPath, nSampleID);
        }

        //-------------------------------------------------------------------------
        room_builder::GCTask::GCTask(room_builder *base)
        {
            pBuilder    = base;
        }

        room_builder::GCTask::~GCTask()
        {
            pBuilder    = NULL;
        }

        status_t room_builder::GCTask::run()
        {
            pBuilder->perform_gc();
            return STATUS_OK;
        }

        void room_builder::GCTask::dump(dspu::IStateDumper *v) const
        {
            v->write("pBuilder", pBuilder);
        }

        //-------------------------------------------------------------------------
        room_builder::room_builder(const meta::plugin_t *metadata, size_t inputs):
            plug::Module(metadata),
            s3DLauncher(this),
            sConfigurator(this),
            sSaver(this),
            sGCTask(this)
        {
            nInputs         = inputs;

            nRenderThreads  = 0;
            fRenderQuality  = 0.5f;
            bRenderNormalize= true;
            enRenderStatus  = STATUS_OK;
            fRenderProgress = 0.0f;
            fRenderCmd      = 0.0f;
            nFftRank        = 0;
            pGCList         = NULL;

            nSceneStatus    = STATUS_UNSPECIFIED;
            fSceneProgress  = 0.0f;
            nSync           = 0;

            pBypass         = NULL;
            pRank           = NULL;
            pDry            = NULL;
            pWet            = NULL;
            pRenderThreads  = NULL;
            pRenderQuality  = NULL;
            pRenderStatus   = NULL;
            pRenderProgress = NULL;
            pRenderNormalize= NULL;
            pRenderCmd      = NULL;
            pOutGain        = NULL;
            pPredelay       = NULL;
            p3DFile         = NULL;
            p3DProgress     = NULL;
            p3DStatus       = NULL;
            p3DOrientation  = NULL;
            pScaleX         = NULL;
            pScaleY         = NULL;
            pScaleZ         = NULL;
            pRenderer       = NULL;

            pData           = NULL;
            pExecutor       = NULL;

            dsp::init_vector_dxyz(&sScale, 1.0f, 1.0f, 1.0f);
        }

        room_builder::~room_builder()
        {
            do_destroy();
        }

        void room_builder::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Pass wrapper
            plug::Module::init(wrapper, ports);

            // Remember executor service
            pExecutor       = wrapper->executor();
            lsp_trace("Executor = %p", pExecutor);

            // Allocate memory
            size_t tmp_buf_size = TMP_BUF_SIZE * sizeof(float);
            size_t thumb_size   = meta::room_builder_metadata::MESH_SIZE *
                                  meta::room_builder_metadata::TRACKS_MAX * sizeof(float);
            size_t alloc        = tmp_buf_size * (meta::room_builder_metadata::CONVOLVERS + 2) +
                                  thumb_size * meta::room_builder_metadata::CAPTURES;
            uint8_t *ptr        = alloc_aligned<uint8_t>(pData, alloc);
            if (pData == NULL)
                return;

            // Initialize 3D loader
            s3DLoader.init(this);

            // Initialize inputs
            for (size_t i=0; i<2; ++i)
            {
                input_t *in     = &vInputs[i];
                in->vIn         = NULL;
                in->pIn         = NULL;
                in->pPan        = NULL;
            }

            // Initialize output channels
            for (size_t i=0; i<2; ++i)
            {
                channel_t *c    = &vChannels[i];

                if (!c->sPlayer.init(meta::room_builder_metadata::CAPTURES, 32))
                    return;
                if (!c->sEqualizer.init(meta::room_builder_metadata::EQ_BANDS + 2, CONV_RANK))
                    return;
                c->sEqualizer.set_mode(dspu::EQM_BYPASS);

                c->fDryPan[0]   = 0.0f;
                c->fDryPan[1]   = 0.0f;

                c->vOut         = NULL;
                c->vBuffer      = reinterpret_cast<float *>(ptr);
                ptr            += tmp_buf_size;

                c->pOut         = NULL;

                c->pWetEq       = NULL;
                c->pLowCut      = NULL;
                c->pLowFreq     = NULL;
                c->pHighCut     = NULL;
                c->pHighFreq    = NULL;

                for (size_t j=0; j<meta::room_builder_metadata::EQ_BANDS; ++j)
                    c->pFreqGain[j]     = NULL;
            }

            // Initialize sources
            for (size_t i=0; i<meta::room_builder_metadata::SOURCES; ++i)
            {
                source_t *src       = &vSources[i];

                src->bEnabled       = false;
                src->enType         = dspu::RT_AS_TRIANGLE;
                dsp::init_point_xyz(&src->sPos, 0.0f, -1.0f, 0.0f);
                src->fYaw           = 0.0f;
                src->fPitch         = 0.0f;
                src->fRoll          = 0.0f;
                src->fSize          = 0.0f;
                src->fHeight        = 0.0f;
                src->fAngle         = 0.0f;
                src->fCurvature     = 1.0f;
                src->fAmplitude     = 1.0f;

                src->pEnabled       = NULL;
                src->pType          = NULL;
                src->pPhase         = NULL;
                src->pPosX          = NULL;
                src->pPosY          = NULL;
                src->pPosZ          = NULL;
                src->pYaw           = NULL;
                src->pPitch         = NULL;
                src->pRoll          = NULL;
                src->pSize          = NULL;
                src->pHeight        = NULL;
                src->pAngle         = NULL;
                src->pCurvature     = NULL;
            }

            // Initialize captures
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *cap      = &vCaptures[i];

                cap->sListen.init();

                dsp::init_point_xyz(&cap->sPos, 0.0f, 1.0f, 0.0f);
                cap->fYaw           = 0.0f;
                cap->fPitch         = 0.0f;
                cap->fRoll          = 0.0f;
                cap->fCapsule       = meta::room_builder_metadata::CAPSULE_DFL;
                cap->sConfig        = dspu::RT_CC_XY;
                cap->fAngle         = meta::room_builder_metadata::ANGLE_DFL;
                cap->fDistance      = meta::room_builder_metadata::DISTANCE_DFL;
                cap->enDirection    = dspu::RT_AC_OMNI;
                cap->enSide         = dspu::RT_AC_BIDIR;

                cap->bEnabled       = (i == 0);
                cap->nRMin          = 1;
                cap->nRMax          = -1;

                cap->fHeadCut       = 0.0f;
                cap->fTailCut       = 0.0f;
                cap->fFadeIn        = 0.0f;
                cap->fFadeOut       = 0.0f;
                cap->bReverse       = false;
                cap->fMakeup        = 1.0f;
                cap->nLength        = 0;
                cap->nStatus        = STATUS_NO_DATA;
                cap->fCurrLen       = 0.0f;
                cap->fMaxLen        = 0.0f;

                cap->bSync          = false;
                cap->bExport        = false;

                cap->pProcessed     = NULL;

                for (size_t j=0; j<meta::room_builder_metadata::TRACKS_MAX; ++j)
                {
                    cap->vThumbs[j]     = reinterpret_cast<float *>(ptr);
                    ptr                += meta::room_builder_metadata::MESH_SIZE * sizeof(float);
                }

                cap->pEnabled       = NULL;
                cap->pRMin          = NULL;
                cap->pRMax          = NULL;
                cap->pPosX          = NULL;
                cap->pPosY          = NULL;
                cap->pPosZ          = NULL;
                cap->pYaw           = NULL;
                cap->pPitch         = NULL;
                cap->pRoll          = NULL;
                cap->pCapsule       = NULL;
                cap->pConfig        = NULL;
                cap->pAngle         = NULL;
                cap->pDistance      = NULL;
                cap->pDirection     = NULL;
                cap->pSide          = NULL;

                cap->pHeadCut       = NULL;
                cap->pTailCut       = NULL;
                cap->pFadeIn        = NULL;
                cap->pFadeOut       = NULL;
                cap->pListen        = NULL;
                cap->pReverse       = NULL;
                cap->pMakeup        = NULL;
                cap->pStatus        = NULL;
                cap->pLength        = NULL;
                cap->pCurrLen       = NULL;
                cap->pMaxLen        = NULL;
                cap->pThumbs        = NULL;

                cap->pOutFile       = NULL;
                cap->pSaveCmd       = NULL;
                cap->pSaveStatus    = NULL;
                cap->pSaveProgress  = NULL;
            }

            // Initialize convolvers
            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                lsp_trace("Binding convolution #%d ports", int(i));
                convolver_t *c  = &vConvolvers[i];

                c->pCurr            = NULL;
                c->pSwap            = NULL;

                c->nSampleID        = 0;
                c->nTrackID         = 0;

                c->vBuffer          = reinterpret_cast<float *>(ptr);
                ptr                += tmp_buf_size;

                c->fPanIn[0]        = 0.0f;
                c->fPanIn[1]        = 0.0f;
                c->fPanOut[0]       = 0.0f;
                c->fPanOut[1]       = 0.0f;

                c->pMakeup          = NULL;
                c->pPanIn           = NULL;
                c->pPanOut          = NULL;
                c->pSample          = NULL;
                c->pTrack           = NULL;
                c->pPredelay        = NULL;
                c->pMute            = NULL;
                c->pActivity        = NULL;
            }

            // Bind ports
            size_t port_id = 0;

            lsp_trace("Binding audio ports");
            for (size_t i=0; i<nInputs; ++i)
            {
                TRACE_PORT(ports[port_id]);
                vInputs[i].pIn      = ports[port_id++];
            }
            for (size_t i=0; i<2; ++i)
            {
                TRACE_PORT(ports[port_id]);
                vChannels[i].pOut   = ports[port_id++];
            }

            // Bind controlling ports
            lsp_trace("Binding common ports");
            TRACE_PORT(ports[port_id]);
            pBypass         = ports[port_id++];
            TRACE_PORT(ports[port_id]);            // Skip view selector
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip editor selector
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip processor selector
            port_id++;
            TRACE_PORT(ports[port_id]);            // FFT rank
            pRank           = ports[port_id++];
            TRACE_PORT(ports[port_id]);            // Pre-delay
            pPredelay       = ports[port_id++];

            for (size_t i=0; i<nInputs; ++i)        // Panning ports
            {
                TRACE_PORT(ports[port_id]);
                vInputs[i].pPan     = ports[port_id++];
            }

            TRACE_PORT(ports[port_id]);
            pDry            = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pWet            = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pOutGain        = ports[port_id++];

            TRACE_PORT(ports[port_id]);
            pRenderThreads  = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pRenderQuality  = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pRenderStatus   = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pRenderProgress = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pRenderNormalize= ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pRenderCmd      = ports[port_id++];

            TRACE_PORT(ports[port_id]);
            p3DFile         = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            p3DStatus       = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            p3DProgress     = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            p3DOrientation  = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pScaleX         = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pScaleY         = ports[port_id++];
            TRACE_PORT(ports[port_id]);
            pScaleZ         = ports[port_id++];

            // Skip camera settings
            TRACE_PORT(ports[port_id]);            // Skip camera x
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip camera y
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip camera z
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip camera yaw
            port_id++;
            TRACE_PORT(ports[port_id]);            // Skip camera pitch
            port_id++;

            // Bind sources
            TRACE_PORT(ports[port_id]);            // Skip source selector
            port_id++;

            for (size_t i=0; i<meta::room_builder_metadata::SOURCES; ++i)
            {
                source_t *src   = &vSources[i];

                TRACE_PORT(ports[port_id]);
                src->pEnabled       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pType          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pPhase         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pPosX          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pPosY          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pPosZ          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pYaw           = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pPitch         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pRoll          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pSize          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pHeight        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pAngle         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                src->pCurvature     = ports[port_id++];

                TRACE_PORT(ports[port_id]);
                port_id++;          // Skip hue value
            }

            // Bind captures
            TRACE_PORT(ports[port_id]);            // Skip capture selector
            port_id++;

            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *cap  = &vCaptures[i];

                TRACE_PORT(ports[port_id]);
                cap->pEnabled       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pRMin          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pRMax          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pPosX          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pPosY          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pPosZ          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pYaw           = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pPitch         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pRoll          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pCapsule       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pConfig        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pAngle         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pDistance      = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pDirection     = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pSide          = ports[port_id++];

                TRACE_PORT(ports[port_id]);
                cap->pHeadCut       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pTailCut       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pFadeIn        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pFadeOut       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pListen        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pReverse       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pMakeup        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pStatus        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pLength        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pCurrLen       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pMaxLen        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pThumbs        = ports[port_id++];

                TRACE_PORT(ports[port_id]);
                cap->pOutFile       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pSaveCmd       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pSaveStatus    = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                cap->pSaveProgress  = ports[port_id++];

                TRACE_PORT(ports[port_id]);
                port_id++;          // Skip hue value
            }

            // Bind convolver ports
            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                lsp_trace("Binding convolution #%d ports", int(i));
                convolver_t *c  = &vConvolvers[i];

                if (nInputs > 1)    // Input panning
                {
                    TRACE_PORT(ports[port_id]);
                    c->pPanIn       = ports[port_id++];
                }

                TRACE_PORT(ports[port_id]);
                c->pSample      = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pTrack       = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pMakeup      = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pMute        = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pActivity    = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pPredelay    = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pPanOut      = ports[port_id++];
            }

            // Bind wet processing ports
            lsp_trace("Binding wet processing ports");
            size_t port         = port_id;
            for (size_t i=0; i<2; ++i)
            {
                channel_t *c        = &vChannels[i];

                TRACE_PORT(ports[port_id]);
                c->pWetEq           = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pLowCut          = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pLowFreq         = ports[port_id++];

                for (size_t j=0; j<meta::room_builder_metadata::EQ_BANDS; ++j)
                {
                    TRACE_PORT(ports[port_id]);
                    c->pFreqGain[j]     = ports[port_id++];
                }

                TRACE_PORT(ports[port_id]);
                c->pHighCut         = ports[port_id++];
                TRACE_PORT(ports[port_id]);
                c->pHighFreq        = ports[port_id++];

                port_id         = port;
            }
        }

        void room_builder::destroy()
        {
            plug::Module::destroy();
            do_destroy();
        }

        void room_builder::do_destroy()
        {
            // Stop active rendering task
            if (pRenderer != NULL)
            {
                pRenderer->terminate();
                pRenderer->join();
                delete pRenderer;
                pRenderer = NULL;
            }

            sScene.destroy();
            s3DLoader.destroy();

            if (pData != NULL)
            {
                free_aligned(pData);
                pData       = NULL;
            }

            // Perform garbage collection
            perform_gc();

            // Destroy captures
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *c    = &vCaptures[i];
                destroy_sample(c->pProcessed);
            }

            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                convolver_t *c  = &vConvolvers[i];
                destroy_convolver(c->pCurr);
                destroy_convolver(c->pSwap);

                c->sDelay.destroy();
            }

            // Destroy channels
            for (size_t i=0; i<2; ++i)
            {
                channel_t *c = &vChannels[i];
                c->sEqualizer.destroy();
                dspu::Sample *gc_list = c->sPlayer.destroy(false);
                destroy_gc_samples(gc_list);
                c->vOut     = NULL;
                c->vBuffer  = NULL;
            }
        }

        void room_builder::destroy_sample(dspu::Sample * &s)
        {
            if (s == NULL)
                return;

            s->destroy();
            delete s;
            lsp_trace("Destroyed sample %p", s);
            s = NULL;
        }

        void room_builder::destroy_convolver(dspu::Convolver * &c)
        {
            if (c == NULL)
                return;

            c->destroy();
            delete c;
            lsp_trace("Destroyed convolver %p", c);
            c = NULL;
        }

        void room_builder::destroy_gc_samples(dspu::Sample *gc_list)
        {
            // Iterate over the list and destroy each sample in the list
            while (gc_list != NULL)
            {
                dspu::Sample *next = gc_list->gc_next();
                destroy_sample(gc_list);
                gc_list = next;
            }
        }

        size_t room_builder::get_fft_rank(size_t rank)
        {
            return meta::room_builder_metadata::FFT_RANK_MIN + rank;
        }

        void room_builder::update_settings()
        {
            float out_gain      = pOutGain->value();
            float dry_gain      = pDry->value() * out_gain;
            float wet_gain      = pWet->value() * out_gain;
            bool bypass         = pBypass->value() >= 0.5f;
            float predelay      = pPredelay->value();
            size_t rank         = get_fft_rank(pRank->value());

            // Adjust FFT rank
            if (rank != nFftRank)
            {
                nFftRank            = rank;
                sConfigurator.query_launch();
            }

            // Adjust size of scene and number of threads to render
            sScale.dx           = pScaleX->value() * 0.01f;
            sScale.dy           = pScaleY->value() * 0.01f;
            sScale.dz           = pScaleZ->value() * 0.01f;
            nRenderThreads      = pRenderThreads->value();
            bRenderNormalize    = pRenderNormalize->value() >= 0.5f;
            fRenderQuality      = pRenderQuality->value() * 0.01f;

            // Check that render request has been triggered
            float old_cmd       = fRenderCmd;
            fRenderCmd          = pRenderCmd->value();
            if ((old_cmd >= 0.5f) && (fRenderCmd < 0.5f))
            {
                lsp_trace("Triggered render request");
                nSync              |= SYNC_TOGGLE_RENDER;
            }

            // Adjust volume of dry channel
            if (nInputs == 1)
            {
                float pan               = vInputs[0].pPan->value();
                vChannels[0].fDryPan[0] = (100.0f - pan) * 0.005f * dry_gain;
                vChannels[0].fDryPan[1] = 0.0f;
                vChannels[1].fDryPan[0] = (100.0f + pan) * 0.005f * dry_gain;
                vChannels[1].fDryPan[1] = 0.0f;
            }
            else
            {
                float pan_l             = vInputs[0].pPan->value();
                float pan_r             = vInputs[1].pPan->value();

                vChannels[0].fDryPan[0] = (100.0f - pan_l) * 0.005f * dry_gain;
                vChannels[0].fDryPan[1] = (100.0f - pan_r) * 0.005f * dry_gain;
                vChannels[1].fDryPan[0] = (100.0f + pan_l) * 0.005f * dry_gain;
                vChannels[1].fDryPan[1] = (100.0f + pan_r) * 0.005f * dry_gain;
            }

            // Update source settings
            for (size_t i=0; i<meta::room_builder_metadata::SOURCES; ++i)
            {
                source_t *src       = &vSources[i];
                src->bEnabled       = src->pEnabled->value() >= 0.5f;
                src->enType         = decode_source_type(src->pType->value());
                src->sPos.x         = src->pPosX->value();
                src->sPos.y         = src->pPosY->value();
                src->sPos.z         = src->pPosZ->value();
                src->sPos.w         = 1.0f;
                src->fYaw           = src->pYaw->value();
                src->fPitch         = src->pPitch->value();
                src->fRoll          = src->pRoll->value();
                src->fSize          = src->pSize->value() * 0.01f;   // cm -> m
                src->fHeight        = src->pHeight->value() * 0.01f; // cm -> m
                src->fAngle         = src->pAngle->value();
                src->fCurvature     = src->pCurvature->value();
                src->fAmplitude     = (src->pPhase->value() >= 0.5f) ? -1.0f : 1.0f;
            }

            // Update capture settings
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *cap      = &vCaptures[i];

                cap->bEnabled       = cap->pEnabled->value() >= 0.5f;
                cap->nRMin          = ssize_t(cap->pRMin->value()) - 1;
                cap->nRMax          = ssize_t(cap->pRMax->value()) - 1;
                cap->sPos.x         = cap->pPosX->value();
                cap->sPos.y         = cap->pPosY->value();
                cap->sPos.z         = cap->pPosZ->value();
                cap->sPos.w         = 1.0f;
                cap->fYaw           = cap->pYaw->value();
                cap->fPitch         = cap->pPitch->value();
                cap->fRoll          = cap->pRoll->value();
                cap->fCapsule       = cap->pCapsule->value() * 0.5f;
                cap->sConfig        = decode_config(cap->pConfig->value());
                cap->fAngle         = cap->pAngle->value();
                cap->fDistance      = cap->pDistance->value();
                cap->enDirection    = decode_direction(cap->pDirection->value());
                cap->enSide         = decode_side_direction(cap->pSide->value());
                cap->fMakeup        = cap->pMakeup->value();

                // Accept changes
                plug::path_t *path        = cap->pOutFile->buffer<plug::path_t>();
                if ((path != NULL) && (path->pending()))
                {
                    path->accept();
                    path->commit();
                }
                if (cap->pSaveCmd->value() >= 0.5f) // Toggle the export flag
                    cap->bExport        = true;

                // Check that we need to synchronize capture settings with convolver
                float hcut      = cap->pHeadCut->value();
                float tcut      = cap->pTailCut->value();
                float fadein    = cap->pFadeIn->value();
                float fadeout   = cap->pFadeOut->value();
                bool  reverse   = cap->pReverse->value() >= 0.5f;

                if ((cap->fHeadCut != hcut) ||
                    (cap->fTailCut != tcut) ||
                    (cap->fFadeIn != fadein) ||
                    (cap->fFadeOut != fadeout) ||
                    (cap->bReverse != reverse))
                {
                    cap->fHeadCut       = hcut;
                    cap->fTailCut       = tcut;
                    cap->fFadeIn        = fadein;
                    cap->fFadeOut       = fadeout;
                    cap->bReverse       = reverse;

                    sConfigurator.query_launch();
                }

                // Listen button pressed?
                if (cap->pListen != NULL)
                    cap->sListen.submit(cap->pListen->value());
            }

            // Adjust channel setup
            for (size_t i=0; i<2; ++i)
            {
                channel_t *c        = &vChannels[i];
                c->sBypass.set_bypass(bypass);
                c->sPlayer.set_gain(out_gain);

                // Update equalization parameters
                dspu::Equalizer *eq             = &c->sEqualizer;
                dspu::equalizer_mode_t eq_mode  = (c->pWetEq->value() >= 0.5f) ? dspu::EQM_IIR : dspu::EQM_BYPASS;
                eq->set_mode(eq_mode);

                if (eq_mode != dspu::EQM_BYPASS)
                {
                    dspu::filter_params_t fp;
                    size_t band     = 0;

                    // Set-up parametric equalizer
                    while (band < meta::room_builder_metadata::EQ_BANDS)
                    {
                        if (band == 0)
                        {
                            fp.fFreq        = band_freqs[band];
                            fp.fFreq2       = fp.fFreq;
                            fp.nType        = dspu::FLT_MT_LRX_LOSHELF;
                        }
                        else if (band == (meta::room_builder_metadata::EQ_BANDS - 1))
                        {
                            fp.fFreq        = band_freqs[band-1];
                            fp.fFreq2       = fp.fFreq;
                            fp.nType        = dspu::FLT_MT_LRX_HISHELF;
                        }
                        else
                        {
                            fp.fFreq        = band_freqs[band-1];
                            fp.fFreq2       = band_freqs[band];
                            fp.nType        = dspu::FLT_MT_LRX_LADDERPASS;
                        }

                        fp.fGain        = c->pFreqGain[band]->value();
                        fp.nSlope       = 2;
                        fp.fQuality     = 0.0f;

                        // Update filter parameters
                        eq->set_params(band++, &fp);
                    }

                    // Setup hi-pass filter
                    size_t hp_slope = c->pLowCut->value() * 2;
                    fp.nType        = (hp_slope > 0) ? dspu::FLT_BT_BWC_HIPASS : dspu::FLT_NONE;
                    fp.fFreq        = c->pLowFreq->value();
                    fp.fFreq2       = fp.fFreq;
                    fp.fGain        = 1.0f;
                    fp.nSlope       = hp_slope;
                    fp.fQuality     = 0.0f;
                    eq->set_params(band++, &fp);

                    // Setup low-pass filter
                    size_t lp_slope = c->pHighCut->value() * 2;
                    fp.nType        = (lp_slope > 0) ? dspu::FLT_BT_BWC_LOPASS : dspu::FLT_NONE;
                    fp.fFreq        = c->pHighFreq->value();
                    fp.fFreq2       = fp.fFreq;
                    fp.fGain        = 1.0f;
                    fp.nSlope       = lp_slope;
                    fp.fQuality     = 0.0f;
                    eq->set_params(band++, &fp);
                }
            }

            // Update settings of convolvers
            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                convolver_t *cv         = &vConvolvers[i];

                // Allow to reconfigure convolver only when configuration task is in idle state
                size_t sampleid         = cv->pSample->value();
                size_t trackid          = cv->pTrack->value();

                if ((cv->nSampleID != sampleid) ||
                    (cv->nTrackID != trackid))
                {
                    cv->nSampleID           = sampleid;
                    cv->nTrackID            = trackid;
                    sConfigurator.query_launch();
                }

                // Apply panning to each convolver
                float smakeup           = (sampleid > 0) ? vCaptures[sampleid-1].fMakeup : 1.0f; // Sample makeup
                float makeup            = (cv->pMute->value() < 0.5f) ? cv->pMakeup->value() * wet_gain * smakeup : 0.0f;
                if (nInputs == 1)
                {
                    cv->fPanIn[0]       = 1.0f;
                    cv->fPanIn[1]       = 0.0f;
                }
                else
                {
                    float pan           = cv->pPanIn->value();
                    cv->fPanIn[0]       = (100.0f - pan) * 0.005f;
                    cv->fPanIn[1]       = (100.0f + pan) * 0.005f;
                }

                float pan           = cv->pPanOut->value();
                cv->fPanOut[0]      = (100.0f - pan) * 0.005f * makeup;
                cv->fPanOut[1]      = (100.0f + pan) * 0.005f * makeup;

                // Set pre-delay
                cv->sDelay.set_delay(dspu::millis_to_samples(fSampleRate, predelay + cv->pPredelay->value()));
            }
        }

        dspu::rt_audio_source_t room_builder::decode_source_type(float value)
        {
            switch (ssize_t(value))
            {
                case 1:     return dspu::RT_AS_TETRA;
                case 2:     return dspu::RT_AS_OCTA;
                case 3:     return dspu::RT_AS_BOX;
                case 4:     return dspu::RT_AS_ICO;
                case 5:     return dspu::RT_AS_CYLINDER;
                case 6:     return dspu::RT_AS_CONE;
                case 7:     return dspu::RT_AS_OCTASPHERE;
                case 8:     return dspu::RT_AS_ICOSPHERE;
                case 9:     return dspu::RT_AS_FSPOT;
                case 10:    return dspu::RT_AS_CSPOT;
                case 11:    return dspu::RT_AS_SSPOT;
                default:    break;
            }
            return dspu::RT_AS_TRIANGLE;
        }

        dspu::rt_capture_config_t room_builder::decode_config(float value)
        {
            switch (ssize_t(value))
            {
                case 1:     return dspu::RT_CC_XY;
                case 2:     return dspu::RT_CC_AB;
                case 3:     return dspu::RT_CC_ORTF;
                case 4:     return dspu::RT_CC_MS;
                default:    break;
            }
            return dspu::RT_CC_MONO;
        }

        dspu::rt_audio_capture_t room_builder::decode_direction(float value)
        {
            switch (ssize_t(value))
            {
                case 1:     return dspu::RT_AC_SCARDIO; break;
                case 2:     return dspu::RT_AC_HCARDIO; break;
                case 3:     return dspu::RT_AC_BIDIR; break;
                case 4:     return dspu::RT_AC_EIGHT; break;
                case 5:     return dspu::RT_AC_OMNI; break;
                default:    break;
            }
            return dspu::RT_AC_CARDIO;
        }

        dspu::rt_audio_capture_t room_builder::decode_side_direction(float value)
        {
            switch (ssize_t(value))
            {
                case 1: return dspu::RT_AC_EIGHT;
                default: break;
            }
            return dspu::RT_AC_BIDIR;
        }

        void room_builder::update_sample_rate(long sr)
        {
            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
                vConvolvers[i].sDelay.init(dspu::millis_to_samples(sr, meta::room_builder_metadata::PREDELAY_MAX * 4.0f));

            for (size_t i=0; i<2; ++i)
            {
                vChannels[i].sBypass.init(sr);
                vChannels[i].sEqualizer.set_sample_rate(sr);
            }

            sConfigurator.query_launch();
        }

        void room_builder::process_render_requests()
        {
            // The render signal is pending?
            if ((nSync & SYNC_TOGGLE_RENDER) && (s3DLauncher.idle()) && (s3DLoader.idle()))
            {
                if (pExecutor->submit(&s3DLauncher))
                {
                    lsp_trace("Successfully submitted Render launcher task");
                    nSync &= ~SYNC_TOGGLE_RENDER;       // Reset render request flag
                }
            }
            else if (s3DLauncher.completed())
            {
                status_t res = s3DLauncher.code();
                if (res != STATUS_OK)
                {
                    fRenderProgress = 0.0f;
                    enRenderStatus  = s3DLauncher.code();
                }
                s3DLauncher.reset();
            }
        }

        void room_builder::process_scene_load_requests()
        {
            // Check the state of input file
            plug::path_t *path      = p3DFile->buffer<plug::path_t>();
            if (path != NULL)
            {
                if ((path->pending()) && (s3DLoader.idle()) && (s3DLauncher.idle())) // There is pending request for 3D file reload
                {
                    // Copy path
                    ::strncpy(s3DLoader.sPath, path->path(), PATH_MAX-1);
                    s3DLoader.nFlags            = path->flags();
                    s3DLoader.sPath[PATH_MAX-1] = '\0';
                    lsp_trace("Submitted scene file %s", s3DLoader.sPath);

                    // Try to submit task
                    if (pExecutor->submit(&s3DLoader))
                    {
                        lsp_trace("Successfully submitted load task");
                        nSceneStatus    = STATUS_LOADING;
                        fSceneProgress  = 0.0f;
                        path->accept();
                    }
                }
                else if ((path->accepted()) && (s3DLoader.completed())) // The reload request has been processed
                {
                    // Update file status and set re-rendering flag
                    nSceneStatus    = s3DLoader.code();
                    fSceneProgress  = 100.0f;

                    sScene.swap(&s3DLoader.sScene);

                    // Now we surely can commit changes and reset task state
                    lsp_trace("File loading task has completed with status %d", int(nSceneStatus));
                    path->commit();
                    s3DLoader.reset();
                }
            }
        }

        void room_builder::process_save_sample_requests()
        {
            if (sSaver.idle())
            {
                // Submit save requests if they are present
                for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
                {
                    capture_t *cap      = &vCaptures[i];
                    if (!cap->bExport)
                        continue;

                    sSaver.bind(i, cap);
                    if (pExecutor->submit(&sSaver))
                    {
                        cap->bExport        = false;
                        cap->pSaveStatus->set_value(STATUS_LOADING);
                        cap->pSaveProgress->set_value(0.0f);
                        break;
                    }
                }
            }
            else if (sSaver.completed())
            {
                capture_t *cap = &vCaptures[sSaver.nSampleID];
                cap->pSaveStatus->set_value(sSaver.code());
                cap->pSaveProgress->set_value(100.0f);

                sSaver.reset();
            }
        }

        void room_builder::process_configuration_requests()
        {
            // Do we need to launch configurator task?
            if ((sConfigurator.idle()) && (sConfigurator.need_launch()))
            {
                // Try to launch configurator
                size_t change_req   = sConfigurator.change_req();

                if (pExecutor->submit(&sConfigurator))
                {
                    sConfigurator.commit(change_req);
                    lsp_trace("Successfully submitted reconfigurator task");
                }
            }
            else if ((sConfigurator.completed()) && (sSaver.idle()))
            {
                lsp_trace("Reconfiguration task has completed with status %d", int(sConfigurator.code()));

                // Commit state of convolvers
                for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
                {
                    convolver_t *c      = &vConvolvers[i];
                    dspu::Convolver *cv = c->pCurr;
                    c->pCurr            = c->pSwap;
                    c->pSwap            = cv;
                }

                for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
                {
                    capture_t  *c   = &vCaptures[i];

                    // Bind sample player
                    for (size_t j=0; j<2; ++j)
                    {
                        channel_t *sc = &vChannels[j];
                        sc->sPlayer.bind(i, c->pProcessed);
                    }

                    c->pProcessed       = NULL;
                    c->bSync            = true;
                }

                // Accept the configurator task
                sConfigurator.reset();
            }
        }

        void room_builder::process_gc_requests()
        {
            if (sGCTask.completed())
                sGCTask.reset();

            if (sGCTask.idle())
            {
                // Obtain the list of samples for destroy
                if (pGCList == NULL)
                {
                    for (size_t i=0; i<2; ++i)
                        if ((pGCList = vChannels[i].sPlayer.gc()) != NULL)
                            break;
                }
                if (pGCList != NULL)
                    pExecutor->submit(&sGCTask);
            }
        }

        void room_builder::process_listen_requests()
        {
            // Update capture settings
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *cap      = &vCaptures[i];

                if (!cap->sListen.pending())
                     continue;

                lsp_trace("Submitted listen toggle");
                dspu::Sample *s = vChannels[0].sPlayer.get(i);
                size_t n_c      = (s != NULL) ? s->channels() : 0;
                if (n_c > 0)
                {
                    for (size_t j=0; j<2; ++j)
                        vChannels[j].sPlayer.play(i, j%n_c, cap->fMakeup, 0);
                }
                cap->sListen.commit();
            }
        }

        void room_builder::perform_convolution(size_t samples)
        {
            // Bind inputs and outputs
            for (size_t i=0; i<nInputs; ++i)
                vInputs[i].vIn      = vInputs[i].pIn->buffer<float>();

            for (size_t i=0; i<2; ++i)
                vChannels[i].vOut   = vChannels[i].pOut->buffer<float>();

            // Process samples
            while (samples > 0)
            {
                // Determine number of samples to process
                size_t to_do        = TMP_BUF_SIZE;
                if (to_do > samples)
                    to_do               = samples;

                // Clear temporary channel buffer
                dsp::fill_zero(vChannels[0].vBuffer, to_do);
                dsp::fill_zero(vChannels[1].vBuffer, to_do);

                // Call convolvers
                for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
                {
                    convolver_t *c      = &vConvolvers[i];

                    // Prepare input buffer: apply panning if present
                    if (nInputs == 1)
                        dsp::copy(c->vBuffer, vInputs[0].vIn, to_do);
                    else
                        dsp::mix_copy2(c->vBuffer, vInputs[0].vIn, vInputs[1].vIn, c->fPanIn[0], c->fPanIn[1], to_do);

                    // Do processing
                    if (c->pCurr != NULL)
                        c->pCurr->process(c->vBuffer, c->vBuffer, to_do);
                    else
                        dsp::fill_zero(c->vBuffer, to_do);
                    c->sDelay.process(c->vBuffer, c->vBuffer, to_do);

                    // Apply processed signal to output channels
                    dsp::fmadd_k3(vChannels[0].vBuffer, c->vBuffer, c->fPanOut[0], to_do);
                    dsp::fmadd_k3(vChannels[1].vBuffer, c->vBuffer, c->fPanOut[1], to_do);
                }

                // Now apply equalization, bypass control and players
                for (size_t i=0; i<2; ++i)
                {
                    channel_t *c        = &vChannels[i];

                    // Apply equalization
                    c->sEqualizer.process(c->vBuffer, c->vBuffer, to_do);

                    // Pass dry sound to output channels
                    if (nInputs == 1)
                        dsp::fmadd_k3(c->vBuffer, vInputs[0].vIn, c->fDryPan[0], to_do);
                    else
                        dsp::mix_add2(c->vBuffer, vInputs[0].vIn, vInputs[1].vIn, c->fDryPan[0], c->fDryPan[1], to_do);

                    // Apply player and bypass
                    c->sPlayer.process(c->vBuffer, c->vBuffer, to_do);
                    c->sBypass.process(c->vOut, vInputs[i%nInputs].vIn, c->vBuffer, to_do);

                    // Update pointers
                    c->vOut            += to_do;
                }

                for (size_t i=0; i<nInputs; ++i)
                    vInputs[i].vIn     += to_do;

                samples            -= to_do;
            }
        }

        void room_builder::output_parameters()
        {
            if (p3DStatus != NULL)
                p3DStatus->set_value(nSceneStatus);
            if (p3DProgress != NULL)
                p3DProgress->set_value(fSceneProgress);
            if (pRenderStatus != NULL)
                pRenderStatus->set_value(enRenderStatus);
            if (pRenderProgress != NULL)
                pRenderProgress->set_value(fRenderProgress);

            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                // Output information about the convolver
                convolver_t *c          = &vConvolvers[i];
                c->pActivity->set_value((c->pCurr != NULL) ? 1.0f : 0.0f);
            }

            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *c            = &vCaptures[i];

                // Output information about the file
                size_t length           = c->nLength;
                c->pLength->set_value(dspu::samples_to_millis(fSampleRate, length));
                c->pCurrLen->set_value(c->fCurrLen);
                c->pMaxLen->set_value(c->fMaxLen);
                c->pStatus->set_value(c->nStatus);

                // Store file dump to mesh
                plug::mesh_t *mesh      = c->pThumbs->buffer<plug::mesh_t>();
                if ((mesh == NULL) || (!mesh->isEmpty()) || (!c->bSync))
                    continue;

                dspu::Sample *active    = vChannels[0].sPlayer.get(i);
                size_t channels         = (active != NULL) ? active->channels() : 0;
                if (channels > 0)
                {
                    // Copy thumbnails
                    for (size_t j=0; j<channels; ++j)
                        dsp::copy(mesh->pvData[j], c->vThumbs[j], meta::room_builder_metadata::MESH_SIZE);
                    mesh->data(channels, meta::room_builder_metadata::MESH_SIZE);
                }
                else
                    mesh->data(0, 0);
                c->bSync            = false;
            }
        }

        void room_builder::process(size_t samples)
        {
            process_render_requests();
            process_scene_load_requests();
            process_save_sample_requests();
            process_listen_requests();
            process_configuration_requests();
            perform_convolution(samples);
            output_parameters();
        }

        status_t room_builder::bind_sources(dspu::RayTrace3D *rt)
        {
            size_t sources = 0;

            for (size_t i=0; i<meta::room_builder_metadata::SOURCES; ++i)
            {
                source_t *src = &vSources[i];
                if (!src->bEnabled)
                    continue;

                // Configure source
                dspu::rt_source_settings_t ss;
                status_t res = rt_configure_source(&ss, src);
                if (res != STATUS_OK)
                    return res;

                // Add source to capture
                res = rt->add_source(&ss);
                if (res != STATUS_OK)
                    return res;

                ++sources;
            }

            return (sources <= 0) ? STATUS_NO_SOURCES : STATUS_OK;
        }

        status_t room_builder::bind_captures(lltl::parray<sample_t> &samples, dspu::RayTrace3D *rt)
        {
            size_t captures = 0;

            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *cap = &vCaptures[i];
                if (!cap->bEnabled)
                    continue;
                else if ((cap->nRMax >= 0) && (cap->nRMax < cap->nRMin)) // Validate nRMin and nRMax
                    continue;

                // Configure capture
                size_t n = 0;
                dspu::rt_capture_settings_t cs[2];
                status_t res = rt_configure_capture(&n, cs, cap);
                if (res != STATUS_OK)
                    return res;

                // Create sample, add to list and initialize
                sample_t *s = new sample_t();
                if (s == NULL)
                    return STATUS_NO_MEM;
                else if (!samples.add(s))
                {
                    delete s;
                    return STATUS_NO_MEM;
                }
                s->nID          = i;
                s->enConfig     = cap->sConfig;
                if (!s->sSample.init(n, 512))
                    return STATUS_NO_MEM;

                // Bind captures to samples
                for (size_t i=0; i<n; ++i)
                {
                    ssize_t cap_id = rt->add_capture(&cs[i]);
                    if (cap_id < 0)
                        return status_t(-cap_id);

                    res = rt->bind_capture(cap_id, &s->sSample, i, cap->nRMin, cap->nRMax);
                    if (res != STATUS_OK)
                        return res;

                    ++captures;
                }
            }

            return (captures <= 0) ? STATUS_NO_CAPTURES : STATUS_OK;
        }

        void room_builder::destroy_samples(lltl::parray<sample_t> &samples)
        {
            for (size_t i=0, n=samples.size(); i<n; ++i)
            {
                sample_t *s = samples.uget(i);
                if (s != NULL)
                {
                    s->sSample.destroy();
                    delete s;
                }
            }
            samples.flush();
        }

        status_t room_builder::bind_scene(core::KVTStorage *kvt, dspu::RayTrace3D *rt)
        {
            // Clone the scene
            dspu::Scene3D *dst = new dspu::Scene3D();
            if (dst == NULL)
                return STATUS_NO_MEM;

            status_t res = dst->clone_from(&sScene);
            if (res != STATUS_OK)
            {
                delete dst;
                return res;
            }

            // Set-up scene
            res = rt->set_scene(dst, true);
            if (res != STATUS_OK)
            {
                dst->destroy();
                delete dst;
                return res;
            }

            // Update object properties
            obj_props_t props;
            char base[0x40];
            dspu::rt::material_t mat;
            dsp::matrix3d_t world;
            dsp::init_matrix3d_scale(&world, sScale.dx, sScale.dy, sScale.dz);

            for (size_t i=0, n=dst->num_objects(); i<n; ++i)
            {
                dspu::Object3D *obj = dst->object(i);
                if (obj == NULL)
                    continue;

                // Read object properties
                sprintf(base, "/scene/object/%d", int(i));
                read_object_properties(&props, base, kvt);

                // Update object matrix and visibility
                build_object_matrix(obj->matrix(), &props, &world);
                obj->set_visible(props.bEnabled);

                // Initialize material
                mat.absorption[0]   = props.fAbsorption[0] * 0.01f; // % -> units
                mat.absorption[1]   = props.fAbsorption[1] * 0.01f; // % -> units
                mat.diffusion[0]    = props.fDiffusion[0];
                mat.diffusion[1]    = props.fDiffusion[1];
                mat.dispersion[0]   = props.fDispersion[0];
                mat.dispersion[1]   = props.fDispersion[1];
                mat.transparency[0] = props.fTransparency[0] * 0.01f; // % -> units
                mat.transparency[1] = props.fTransparency[1] * 0.01f; // % -> units
                mat.permeability    = props.fSndSpeed / LSP_DSP_UNITS_SOUND_SPEED_M_S;

                // Commit material properties
                res = rt->set_material(i, &mat);
                if (res != STATUS_OK)
                    return res;
            }

            return STATUS_OK;
        }

        status_t room_builder::progress_callback(float progress, void *ptr)
        {
            room_builder *_this    = reinterpret_cast<room_builder *>(ptr);
            _this->enRenderStatus       = STATUS_IN_PROCESS;
            _this->fRenderProgress      = progress * 100.0f;    // Update the progress value
            return STATUS_OK;
        }

        status_t room_builder::start_rendering()
        {
            // Terminate previous thread (if active)
            if (pRenderer != NULL)
            {
                bool finished = pRenderer->finished();

                pRenderer->terminate();
                pRenderer->join();
                delete pRenderer;
                pRenderer = NULL;

                // Current task has been cancelled?
                if (!finished)
                {
                    fRenderProgress = 0;
                    enRenderStatus  = STATUS_CANCELLED;
                    return STATUS_OK;
                }
            }

            // Create raytracing object and initialize with basic values
            dspu::RayTrace3D *rt = new dspu::RayTrace3D();
            if (rt == NULL)
                return STATUS_NO_MEM;

            status_t res = rt->init();
            if (res != STATUS_OK)
            {
                rt->destroy(false);
                delete rt;
                return res;
            }

            rt->set_sample_rate(fSampleRate);

            float energy    = 1e-3f * expf(-4.0f * M_LN10 * fRenderQuality);    // 1e-3 .. 1e-7
            float tolerance = 1e-4f * expf(-2.0f * M_LN10 * fRenderQuality);    // 1e-4 .. 1e-6
            float details   = 1e-8f * expf(-2.0f * M_LN10 * fRenderQuality);    // 1e-8 .. 1e-10

            rt->set_energy_threshold(energy);
            rt->set_tolerance(tolerance);
            rt->set_detalization(details);
            rt->set_normalize(bRenderNormalize);
            rt->set_progress_callback(progress_callback, this);

            // Bind scene to the raytracing
            core::KVTStorage *kvt = kvt_lock();
            if (kvt != NULL)
            {
                lsp_finally { kvt_release(); };

                if ((res = bind_scene(kvt, rt)) != STATUS_OK)
                {
                    rt->destroy(true);
                    delete rt;
                    return res;
                }
            }

            // Bind sources
            if ((res = bind_sources(rt)) != STATUS_OK)
            {
                rt->destroy(true);
                delete rt;
                return res;
            }

            // Bind captures
            lltl::parray<sample_t> samples;
            if ((res = bind_captures(samples, rt)) != STATUS_OK)
            {
                destroy_samples(samples);
                rt->destroy(true);
                delete rt;
                return res;
            }

            // Create renderer and start execution
            pRenderer = new Renderer(this, rt, nRenderThreads, samples);
            if (pRenderer == NULL)
            {
                destroy_samples(samples);
                rt->destroy(true);
                delete rt;
                return STATUS_NO_MEM;
            }

            if ((res = pRenderer->start()) != STATUS_OK)
            {
                delete pRenderer;
                pRenderer = NULL;
                destroy_samples(samples);
                rt->destroy(true);
                delete rt;
                return res;
            }

            // All seems to be OK
            return STATUS_OK;
        }

        void room_builder::perform_gc()
        {
            dspu::Sample *gc_list = lsp::atomic_swap(&pGCList, NULL);
            destroy_gc_samples(gc_list);
        }

        status_t room_builder::commit_samples(lltl::parray<sample_t> &samples)
        {
            // Put each sample to KVT and toggle the reload flag
            core::kvt_param_t p;
            char path[0x40];

            for (size_t i=0, n=samples.size(); i<n; ++i)
            {
                sample_t *s     = samples.uget(i);
                if (s == NULL)
                    continue;

                // Create sample data
                size_t slen         = s->sSample.length();
                size_t payload      = sizeof(dspu::sample_header_t) + slen * s->sSample.channels() * sizeof(float);
                dspu::sample_header_t *hdr = reinterpret_cast<dspu::sample_header_t *>(::malloc(payload));
                if (hdr == NULL)
                    return STATUS_NO_MEM;
                hdr->version        = __IF_LEBE(0, 1);
                hdr->channels       = s->sSample.channels();
                hdr->sample_rate    = fSampleRate;
                hdr->samples        = s->sSample.length();

                hdr->version        = CPU_TO_BE(hdr->version);
                hdr->channels       = CPU_TO_BE(hdr->channels);
                hdr->sample_rate    = CPU_TO_BE(hdr->sample_rate);
                hdr->samples        = CPU_TO_BE(hdr->samples);

                float *fdst         = reinterpret_cast<float *>(&hdr[1]);
                for (size_t i=0; i<s->sSample.channels(); ++i, fdst += slen)
                    ::memcpy(fdst, s->sSample.channel(i), slen * sizeof(float));

                // Post-process Mid/Side audio data
                if (s->enConfig == dspu::RT_CC_MS)
                {
                    float *l            = reinterpret_cast<float *>(&hdr[1]);
                    float *r            = &l[slen];
                    dsp::ms_to_lr(l, r, l, r, slen);
                }

                // Create KVT parameter
                p.type          = core::KVT_BLOB;
                p.blob.ctype    = ::strdup(AUDIO_SAMPLE_CONTENT_TYPE);
                if (p.blob.ctype == NULL)
                {
                    ::free(hdr);
                    return STATUS_NO_MEM;
                }
                p.blob.size     = payload;
                p.blob.data     = hdr;

                // Deploy KVT parameter
                sprintf(path, "/samples/%d", int(s->nID));
                core::KVTStorage *kvt = kvt_lock();
                if (kvt != NULL)
                {
                    kvt->put(path, &p, core::KVT_PRIVATE | core::KVT_DELEGATE); // Delegate memory management to KVT Storage
                    kvt->gc();
                    kvt_release();
                }
                else
                    return STATUS_BAD_STATE;

                // Toggle configurator launch
                sConfigurator.query_launch();
            }

            return STATUS_OK;
        }

        status_t room_builder::reconfigure()
        {
            status_t res;

            // Re-render samples
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
            {
                capture_t *c    = &vCaptures[i];
                destroy_sample(c->pProcessed);

                // Update status and commit request
                c->nStatus      = STATUS_OK;

                // Lock KVT and fetch sample data
                core::KVTStorage *kvt = kvt_lock();
                if (kvt == NULL)
                {
                    c->nStatus      = STATUS_BAD_STATE;
                    continue;
                }
                lsp_finally { kvt_release(); };

                // Fetch KVT sample
                dspu::sample_header_t hdr;
                const float *samples;
                res = fetch_kvt_sample(kvt, i, &hdr, &samples);
                if (res != STATUS_OK)
                {
                    c->nStatus      = res;
                    continue;
                }

                // Allocate new sample
                dspu::Sample *s     = new dspu::Sample();
                if (s == NULL)
                {
                    c->nStatus      = STATUS_NO_MEM;
                    continue;
                }
                lsp_finally { destroy_sample(s); };

                c->nLength          = hdr.samples;
                c->fMaxLen          = dspu::samples_to_millis(hdr.sample_rate, hdr.samples);
                lsp_trace("Allocated sample=%p, original length=%d samples", s, int(c->nLength));

                // Initialize sample
                if (!s->init(hdr.channels, hdr.samples, hdr.samples))
                {
                    c->nStatus      = STATUS_NO_MEM;
                    continue;
                }

                // Sample is present, check boundaries
                size_t head_cut     = dspu::millis_to_samples(fSampleRate, c->fHeadCut);
                size_t tail_cut     = dspu::millis_to_samples(fSampleRate, c->fTailCut);
                ssize_t fsamples    = hdr.samples - head_cut - tail_cut;
                if (fsamples <= 0)
                {
                    s->set_length(0);
                    c->fCurrLen         = 0.0f;

                    for (size_t j=0; j<hdr.channels; ++j)
                        dsp::fill_zero(c->vThumbs[j], meta::room_builder_metadata::MESH_SIZE);
                    continue;
                }
                c->fCurrLen         = dspu::samples_to_millis(hdr.sample_rate, fsamples);

                // Render the sample
                float norm          = 0.0f;
                for (size_t j=0; j<hdr.channels; ++j)
                {
                    const float *src    = &samples[j * hdr.samples];
                    float *dst          = s->channel(j);

                    // Get normalizing factor
                    float xnorm         = dsp::abs_max(src, hdr.samples);
                    if (xnorm > norm)
                        norm            = xnorm;

                    // Copy sample data and apply fading
                    if (c->bReverse)
                        dsp::reverse2(dst, &src[tail_cut], fsamples);
                    else
                        dsp::copy(dst, &src[head_cut], fsamples);
                    if ((hdr.version & 1) != __IF_LEBE(0, 1)) // Endianess does not match?
                        byte_swap(dst, fsamples);

                    dspu::fade_in(dst, dst, dspu::millis_to_samples(fSampleRate, c->fFadeIn), fsamples);
                    dspu::fade_out(dst, dst, dspu::millis_to_samples(fSampleRate, c->fFadeOut), fsamples);

                    // Now render thumbnail
                    src                 = dst;
                    dst                 = c->vThumbs[j];
                    for (size_t k=0; k<meta::room_builder_metadata::MESH_SIZE; ++k)
                    {
                        size_t first    = (k * fsamples) / meta::room_builder_metadata::MESH_SIZE;
                        size_t last     = ((k + 1) * fsamples) / meta::room_builder_metadata::MESH_SIZE;
                        if (first < last)
                            dst[k]          = dsp::abs_max(&src[first], last - first);
                        else
                            dst[k]          = fabs(src[first]);
                    }
                }

                // Normalize graph if possible
                if (norm != 0.0f)
                {
                    norm    = 1.0f / norm;
                    for (size_t j=0; j<hdr.channels; ++j)
                        dsp::mul_k2(c->vThumbs[j], norm, meta::room_builder_metadata::MESH_SIZE);
                }

                // Commit result
                lsp::swap(c->pProcessed, s);
            }

            // Randomize phase of the convolver
            uint32_t phase  = seed_addr(this);
            phase           = ((phase << 16) | (phase >> 16)) & 0x7fffffff;
            uint32_t step   = 0x80000000 / (meta::room_builder_metadata::CONVOLVERS + 1);

            // Reconfigure convolvers
            for (size_t i=0; i<meta::room_builder_metadata::CONVOLVERS; ++i)
            {
                convolver_t *c  = &vConvolvers[i];
                destroy_convolver(c->pSwap);

                // Check that routing has changed
                size_t capture  = c->nSampleID;
                size_t track    = c->nTrackID;
                if ((capture <= 0) || (capture > meta::room_builder_metadata::CAPTURES))
                    continue;
                else
                    --capture;

                // Analyze sample
                dspu::Sample *s = vCaptures[capture].pProcessed;
                if ((s == NULL) || (!s->valid()) || (s->channels() <= track))
                    continue;

                // Now we can create convolver
                dspu::Convolver *cv   = new dspu::Convolver();
                if (cv == NULL)
                    continue;
                lsp_finally { destroy_convolver(cv); };

                if (!cv->init(s->channel(track), s->length(), nFftRank, float((phase + i*step)& 0x7fffffff)/float(0x80000000)))
                    return STATUS_NO_MEM;

                lsp_trace("Allocated convolver pSwap=%p for channel %d (pCurr=%p)", cv, int(i), c->pCurr);
                lsp::swap(c->pSwap, cv);
            }

            return STATUS_OK;
        }

        status_t room_builder::fetch_kvt_sample(core::KVTStorage *kvt, size_t sample_id, dspu::sample_header_t *hdr, const float **samples)
        {
            status_t res;
            const core::kvt_param_t *p;
            const dspu::sample_header_t *phdr;
            char path[0x40];

            sprintf(path, "/samples/%d", int(sample_id));

            // Fetch parameter
            res = kvt->get(path, &p, core::KVT_BLOB);
            if ((res != STATUS_OK) ||
                (p == NULL))
                return STATUS_NO_DATA;

            // Validate blob settings
            if ((p->blob.ctype == NULL) ||
                (p->blob.data == NULL) ||
                (p->blob.size < sizeof(dspu::sample_header_t)) ||
                (::strcmp(p->blob.ctype, AUDIO_SAMPLE_CONTENT_TYPE) != 0))
                return STATUS_CORRUPTED;

            // Decode sample data
            phdr                = reinterpret_cast<const dspu::sample_header_t *>(p->blob.data);
            hdr->version        = BE_TO_CPU(phdr->version);
            hdr->channels       = BE_TO_CPU(phdr->channels);
            hdr->sample_rate    = BE_TO_CPU(phdr->sample_rate);
            hdr->samples        = BE_TO_CPU(phdr->samples);

            if (((hdr->version >> 1) != 0) ||
                ((hdr->samples * hdr->channels * sizeof(float) + sizeof(dspu::sample_header_t)) != p->blob.size))
                return STATUS_CORRUPTED;

            *samples            = reinterpret_cast<const float *>(&phdr[1]);
            return STATUS_OK;
        }

        status_t room_builder::save_sample(const char *path, size_t sample_id)
        {
            if (::strlen(path) <= 0)
                return STATUS_BAD_PATH;

            LSPString sp, lspc;
            if (!sp.set_utf8(path))
                return STATUS_NO_MEM;
            if (!lspc.set_ascii(".lspc"))
                return STATUS_NO_MEM;

            // Lock KVT storage
            core::KVTStorage *kvt = kvt_lock();
            if (kvt == NULL)
                return STATUS_BAD_STATE;

            dspu::sample_header_t hdr;
            const float *samples;
            status_t res    = fetch_kvt_sample(kvt, sample_id, &hdr, &samples);

            // Check the extension of file
            if (sp.ends_with_nocase(&lspc))
            {
                lspc::audio_parameters_t params;
                params.channels         = hdr.channels;
                params.sample_format    = (hdr.version & 1) ? lspc::SAMPLE_FMT_F32BE : lspc::SAMPLE_FMT_F32LE;
                params.sample_rate      = hdr.sample_rate;
                params.codec            = lspc::CODEC_PCM;
                params.frames           = hdr.samples;

                // Initialize sample array
                const float **vs        = reinterpret_cast<const float **>(::malloc(params.channels * sizeof(float *)));
                if (vs == NULL)
                {
                    kvt_release();
                    return STATUS_NO_MEM;
                }
                for (size_t i=0; i<params.channels; ++i)
                    vs[i]               = &samples[i * params.frames];

                // Create LSPC writer
                lspc::AudioWriter wr;
                res = wr.create(&sp, &params);
                if (res != STATUS_OK)
                {
                    ::free(vs);
                    kvt_release();
                    return res;
                }

                // Write all samples to the file
                res = wr.write_samples(vs, params.frames);
                res = update_status(res, wr.close());
                ::free(vs);
            }
            else
            {
                dspu::Sample af;
                if (!af.init(hdr.channels, hdr.samples, hdr.samples))
                {
                    kvt_release();
                    return res;
                }

                // Prepare file contents
                for (size_t i=0; i<hdr.channels; ++i)
                {
                    float *dst = af.channel(i);
                    dsp::copy(dst, &samples[i * hdr.samples], hdr.samples);
                    if ((hdr.version & 1) != __IF_LEBE(0, 1))
                        byte_swap(dst, hdr.samples);
                }

                // Store file contents
                af.set_sample_rate(hdr.sample_rate);
                res     = af.save(&sp);
                af.destroy();
                res     = (res < 0) ? -res : STATUS_OK;
            }

            // Release KVT storage and return result
            kvt_release();
            return res;
        }

        void room_builder::state_loaded()
        {
            sConfigurator.query_launch();
        }

        void room_builder::ui_activated()
        {
            // Synchronize thumbnails with UI
            for (size_t i=0; i<meta::room_builder_metadata::CAPTURES; ++i)
                vCaptures[i].bSync  = true;
        }

        void room_builder::kvt_cleanup_objects(core::KVTStorage *kvt, size_t objects)
        {
            core::KVTIterator *it = kvt->enum_branch("/scene/object");
            while (it->next() == STATUS_OK)
            {
                const char *id = it->id();
                if (id == NULL)
                    continue;

                // Must be a pure object identifier
                errno = 0;
                char *endptr;
                long value = ::strtol(id, &endptr, 10);
                if ((errno != 0) || (size_t(endptr - id) != size_t(::strlen(id))))
                    continue;

                // Remove the object
                if ((value < 0) || (value >= ssize_t(objects)))
                {
                    lsp_trace("Removing KVT parameters from %s", it->name());
                    it->remove_branch();
                }
            }
        }

        void room_builder::read_object_properties(obj_props_t *props, const char *base, core::KVTStorage *kvt)
        {
            float enabled;

            kvt_fetch(kvt, base, "name", &props->sName, "unnamed");
            kvt_fetch(kvt, base, "enabled", &enabled, 1.0f);
            kvt_fetch(kvt, base, "center/x", &props->sCenter.x, 0.0f);
            kvt_fetch(kvt, base, "center/y", &props->sCenter.y, 0.0f);
            kvt_fetch(kvt, base, "center/z", &props->sCenter.z, 0.0f);
            kvt_fetch(kvt, base, "position/x", &props->sMove.dx, 0.0f);
            kvt_fetch(kvt, base, "position/y", &props->sMove.dy, 0.0f);
            kvt_fetch(kvt, base, "position/z", &props->sMove.dz, 0.0f);
            kvt_fetch(kvt, base, "rotation/yaw", &props->fYaw, 0.0f);
            kvt_fetch(kvt, base, "rotation/pitch", &props->fPitch, 0.0f);
            kvt_fetch(kvt, base, "rotation/roll", &props->fRoll, 0.0f);
            kvt_fetch(kvt, base, "scale/x", &props->sScale.dx, 1.0f);
            kvt_fetch(kvt, base, "scale/y", &props->sScale.dy, 1.0f);
            kvt_fetch(kvt, base, "scale/z", &props->sScale.dz, 1.0f);
            kvt_fetch(kvt, base, "color/hue", &props->fHue, 0.0f);

            kvt_fetch(kvt, base, "material/absorption/outer", &props->fAbsorption[0], 1.5f);
            kvt_fetch(kvt, base, "material/dispersion/outer", &props->fDispersion[0], 1.0f);
            kvt_fetch(kvt, base, "material/dissipation/outer", &props->fDiffusion[0], 1.0f);
            kvt_fetch(kvt, base, "material/transparency/outer", &props->fTransparency[0], 48.0f);

            kvt_fetch(kvt, base, "material/absorption/inner", &props->fAbsorption[1], 1.5f);
            kvt_fetch(kvt, base, "material/dispersion/inner", &props->fDispersion[1], 1.0f);
            kvt_fetch(kvt, base, "material/diffusion/inner", &props->fDiffusion[1], 1.0f);
            kvt_fetch(kvt, base, "material/transparency/inner", &props->fTransparency[1], 52.0f);

            kvt_fetch(kvt, base, "material/absorption/link", &props->lnkAbsorption, 1.0f);
            kvt_fetch(kvt, base, "material/dispersion/link", &props->lnkDispersion, 1.0f);
            kvt_fetch(kvt, base, "material/diffusion/link", &props->lnkDiffusion, 1.0f);
            kvt_fetch(kvt, base, "material/transparency/link", &props->lnkTransparency, 1.0f);

            kvt_fetch(kvt, base, "material/sound_speed", &props->fSndSpeed, 4250.0f);

            props->bEnabled = (enabled >= 0.5f);
        }

        void room_builder::build_object_matrix(dsp::matrix3d_t *m, const obj_props_t *props, const dsp::matrix3d_t *world)
        {
            dsp::matrix3d_t tmp;

            // Copy world matrix
            *m  = *world;

            // Apply translation
            dsp::init_matrix3d_translate(&tmp,
                    props->sCenter.x + props->sMove.dx,
                    props->sCenter.y + props->sMove.dy,
                    props->sCenter.z + props->sMove.dz
            );
            dsp::apply_matrix3d_mm1(m, &tmp);

            // Apply rotation
            dsp::init_matrix3d_rotate_z(&tmp, props->fYaw * M_PI / 180.0f);
            dsp::apply_matrix3d_mm1(m, &tmp);

            dsp::init_matrix3d_rotate_y(&tmp, props->fPitch * M_PI / 180.0f);
            dsp::apply_matrix3d_mm1(m, &tmp);

            dsp::init_matrix3d_rotate_x(&tmp, props->fRoll * M_PI / 180.0f);
            dsp::apply_matrix3d_mm1(m, &tmp);

            // Apply scale
            dsp::init_matrix3d_scale(&tmp, props->sScale.dx * 0.01f, props->sScale.dy * 0.01f, props->sScale.dz * 0.01f);
            dsp::apply_matrix3d_mm1(m, &tmp);

            // Move center to (0, 0, 0) point
            dsp::init_matrix3d_translate(&tmp, -props->sCenter.x, -props->sCenter.y, -props->sCenter.z);
            dsp::apply_matrix3d_mm1(m, &tmp);
        }

    } /* namespace plugins */
} /* namespace lsp */


