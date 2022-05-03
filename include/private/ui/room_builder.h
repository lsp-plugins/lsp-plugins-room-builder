/*
 * Copyright (C) 2021 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2021 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-room-builder
 * Created on: 12 авг. 2021 г.
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

#ifndef PRIVATE_UI_ROOM_BUILDER_H_
#define PRIVATE_UI_ROOM_BUILDER_H_

#include <lsp-plug.in/plug-fw/ui.h>
#include <lsp-plug.in/plug-fw/core/KVTStorage.h>

namespace lsp
{
    namespace plugui
    {
        /**
         * UI for Parametric Equalizer plugin series
         */
        class room_builder_ui: public ui::Module
        {
            protected:
                class CtlFloatPort: public ui::IPort, public ui::IKVTListener
                {
                    protected:
                        room_builder_ui    *pUI;
                        const char         *sPattern;
                        float               fValue;

                    public:
                        explicit CtlFloatPort(room_builder_ui *ui, const char *pattern, const meta::port_t *meta);
                        virtual ~CtlFloatPort();

                    public:
                        virtual float       value();
                        virtual void        set_value(float value);
                        virtual bool        changed(core::KVTStorage *storage, const char *id, const core::kvt_param_t *value);
                };

                class CtlListPort: public ui::IPort, public ui::IKVTListener
                {
                    protected:
                        room_builder_ui            *pUI;
                        meta::port_t                sMetadata;
                        meta::port_item_t          *pItems;
                        size_t                      nCapacity;
                        size_t                      nItems;
                        lltl::parray<ui::IPort>     vKvtPorts;
                        ssize_t                     nSelectedReq;

                    protected:
                        void set_list_item(size_t id, const char *value);

                    public:
                        explicit CtlListPort(room_builder_ui *ui, const meta::port_t *meta);
                        virtual ~CtlListPort();

                    public:
                        virtual float       value();
                        virtual void        set_value(float value);
                        virtual bool        changed(core::KVTStorage *storage, const char *id, const core::kvt_param_t *value);

                        void                add_port(ui::IPort *port);
                };

                class CtlMaterialPreset: public ui::IPortListener
                {
                    protected:
                        room_builder_ui    *pUI;
                        tk::ComboBox       *pCBox;
                        tk::handler_id_t    hHandler;
                        ui::IPort          *pSpeed;
                        ui::IPort          *pAbsorption;
                        ui::IPort          *pSelected;

                    public:
                        explicit CtlMaterialPreset(room_builder_ui *ui);
                        virtual ~CtlMaterialPreset();

                        void init(const char *preset, const char *selected, const char *speed, const char *absorption);

                    public:
                        virtual void        notify(ui::IPort *port);

                        static status_t     slot_submit(tk::Widget *sender, void *ptr, void *data);
                };

                class CtlKnobBinding: public ui::IPortListener
                {
                    protected:
                        room_builder_ui    *pUI;
                        ui::IPort          *pOuter;
                        ui::IPort          *pInner;
                        ui::IPort          *pLink;
                        bool                bReverse;

                    public:
                        explicit CtlKnobBinding(room_builder_ui *ui, bool reverse);
                        virtual ~CtlKnobBinding();

                        void init(const char *outer, const char *inner, const char *link);

                    public:
                        virtual void        notify(ui::IPort *port);
                };

            protected:
                ssize_t                 nSelected;
                CtlMaterialPreset       sPresets;
                CtlKnobBinding          sAbsorption;
                CtlKnobBinding          sTransparency;
                CtlKnobBinding          sDispersion;
                CtlKnobBinding          sDiffuse;

            protected:
                static void             kvt_cleanup_objects(core::KVTStorage *kvt, size_t objects);

            public:
                explicit room_builder_ui(const meta::plugin_t *metadata);
                virtual ~room_builder_ui();

                virtual status_t        init(ui::IWrapper *wrapper, tk::Display *dpy);

            public:
                virtual status_t        post_init();
        };
    } // namespace plugins
} // namespace lsp


#endif /* PRIVATE_UI_ROOM_BUILDER_H_ */
