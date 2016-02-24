/*
   Copyright (C) 2016 Preet Desai (preet.desai@gmail.com)

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

#include <ks/gl/KsGLCommands.hpp>
#include <ks/gl/KsGLImplementation.hpp>
#include <ks/gui/KsGuiWindow.hpp>
#include <ks/gui/KsGuiApplication.hpp>
#include <ks/platform/KsPlatformMain.hpp>

#include "GrContext.h"
#include "SkCanvas.h"
#include "SkRandom.h"
#include "SkSurface.h"
#include "gl/GrGLInterface.h"
#include "gl/GrGLUtil.h"

using namespace ks;

namespace
{
    class Scene : public ks::Object
    {
    public:
        using base_type = ks::Object;

        Scene(ks::Object::Key const &key,
              shared_ptr<EventLoop> evl,
              gui::Window* win0) :
            ks::Object(key,evl),
            m_win0(win0)
        {
            m_render_task_fn0 =
                    [this](){
                        if(m_win0->SetContextCurrent())
                        {
//                            gl::Clear(gl::ColorBufferBit);

                            SkCanvas* canvas = m_skia_surface->getCanvas();
                            canvas->clear(SK_ColorWHITE);

                            SkPaint paint;
                            paint.setColor(SK_ColorBLUE);
                            paint.setAntiAlias(true);

                            const SkScalar scale = 256.0f;
                            const SkScalar R = 0.45f * scale;
                            const SkScalar TAU = 6.2831853f;
                            SkPath path;
                            path.moveTo(R + 200.0f, 0.0f + 200.0f);
                            for (int i = 1; i < 7; ++i) {
                                SkScalar theta = 3 * i * TAU / 7;
                                path.lineTo((R * cos(theta))+200.0f, (R * sin(theta))+200.0f);
                            }
                            path.close();
                            canvas->drawPath(path,paint);


                            SkRect rect = SkRect::MakeXYWH(300,300,100,100);
                            paint.setColor(SK_ColorRED);
                            canvas->drawRect(rect,paint);

                            canvas->flush();

                            m_win0->SwapBuffers();
                        }
                    };
        }

        void Init(ks::Object::Key const &,
                  shared_ptr<Scene> const &)
        {}

        ~Scene()
        {}

        void OnAppInit()
        {
            m_running = true;
            signal_app_process_events.Emit();

            // Capture GL implementation info
            auto init_gl_task =
                    make_shared<ks::Task>(
                        [this](){
                            if(m_win0->SetContextCurrent())
                            {
                                ks::gl::Implementation::GLCapture();
                            }
                        });

            m_win0->GetEventLoop()->PostTask(init_gl_task);
            init_gl_task->Wait();


            // Set the GL Clear color state
            m_gl_state_set0 = make_unique<gl::StateSet>();

            auto set_clear_color =
                    make_shared<ks::Task>(
                        [this](){
                            if(m_win0->SetContextCurrent())
                            {
                                m_gl_state_set0->CaptureState();
                                m_gl_state_set0->SetClearColor(0.1,0.15,0.15,1);
                            }

                        });

            m_win0->GetEventLoop()->PostTask(set_clear_color);
            set_clear_color->Wait();


            // Init Skia task
            auto init_skia =
                    make_shared<ks::Task>(
                        [this](){
                            if(m_win0->SetContextCurrent())
                            {
                                // interface
                                m_skia_interface.reset(GrGLCreateNativeInterface());
                                m_skia_interface.reset(GrGLInterfaceRemoveNVPR(m_skia_interface));

                                // context
                                m_skia_context.reset(
                                            GrContext::Create(
                                                kOpenGL_GrBackend,
                                                (GrBackendContext)m_skia_interface.get()));

                                // framebuffer object id
                                GrGLint buffer;
                                GR_GL_GetIntegerv(m_skia_interface,GR_GL_FRAMEBUFFER_BINDING,&buffer);

                                // render target
                                GrBackendRenderTargetDesc desc;
                                desc.fWidth = m_win0->size.Get().first;
                                desc.fHeight = m_win0->size.Get().second;
                                desc.fConfig = kSkia8888_GrPixelConfig;
                                desc.fOrigin = kBottomLeft_GrSurfaceOrigin;
                                desc.fSampleCnt = 0;
                                desc.fStencilBits = 8;
                                desc.fRenderTargetHandle = buffer;

                                m_skia_render_target.reset(
                                            m_skia_context->
                                            textureProvider()->
                                            wrapBackendRenderTarget(desc));

                                m_skia_surface.reset(
                                            SkSurface::NewRenderTargetDirect(
                                                m_skia_render_target));
                            }
                        });

            m_win0->GetEventLoop()->PostTask(init_skia);
            init_skia->Wait();
        }

        void OnAppPause()
        {
            m_running = false;
        }

        void OnAppResume()
        {
            m_running = true;
            signal_app_process_events.Emit();
        }

        void OnAppQuit()
        {
            m_running = false;
        }

        void OnAppProcEvents(bool)
        {
            if(m_running)
            {
                auto render_task0 = make_shared<ks::Task>(m_render_task_fn0);
                m_win0->GetEventLoop()->PostTask(render_task0);
                render_task0->Wait();

                // We're assuming vsync is true and will introduce
                // a delay so we schedule the next ProcessEvents task
                // immediately
                signal_app_process_events.Emit();
            }
        }

        Signal<> signal_app_process_events;


    private:
        std::atomic<bool> m_running;
        gui::Window* m_win0;
        std::function<void()> m_render_task_fn0;
        unique_ptr<gl::StateSet> m_gl_state_set0;

        // skia
        SkAutoTUnref<GrGLInterface const> m_skia_interface;
        SkAutoTUnref<GrContext> m_skia_context;
        SkAutoTUnref<GrRenderTarget> m_skia_render_target;
        SkAutoTUnref<SkSurface> m_skia_surface;
    };
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // Create application
    auto app = make_object<gui::Application>();

    // Create windows
    gui::Window::Attributes win_attribs;
    gui::Window::Properties win_props;
    win_props.width = 640;
    win_props.height = 480;
    win_props.swap_interval = 1;

    win_props.title = "Window 0";
    auto win0_render_evl = make_shared<EventLoop>();
    auto win0_render_thread = EventLoop::LaunchInThread(win0_render_evl);
    auto win0 = app->CreateWindow(win0_render_evl,win_attribs,win_props);

    // Create scene
    auto scene =
            make_object<Scene>(
                app->GetEventLoop(),
                win0.get());

    // Setup connections

    // Application ---> Scene
    app->signal_init.Connect(
                scene,
                &Scene::OnAppInit);

    app->signal_pause.Connect(
                scene,
                &Scene::OnAppPause,
                ks::ConnectionType::Direct);

    app->signal_resume.Connect(
                scene,
                &Scene::OnAppResume);

    app->signal_quit.Connect(
                scene,
                &Scene::OnAppQuit,
                ks::ConnectionType::Direct);

    app->signal_processed_events->Connect(
                scene,
                &Scene::OnAppProcEvents);


    // Scene ---> Application
    scene->signal_app_process_events.Connect(
                app,
                &gui::Application::ProcessEvents);


    // Run!
    app->Run();

    // Clean up threads
    EventLoop::RemoveFromThread(win0_render_evl,win0_render_thread,true);

    return 0;
}
