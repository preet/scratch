#include <cstdlib>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <sstream>
#include <sys/time.h>

#include <SDL2/SDL.h>

#ifndef ENV_ANDROID_SDL
#include <glad/glad.h>
#include <SDL2/SDL_opengl.h>
#endif


#if defined(ENV_ANDROID_SDL)
    #include <GLES2/gl2.h>
    #include <GLES2/gl2ext.h>
    #include <android/log.h>
    #include <jni.h>
    #include <SDL2/SDL_main.h>
#endif


// directly based on
// http://lazyfoo.net/tutorials/SDL/51_SDL_and_modern_opengl/index.php

unsigned int g_win_width = 768;
unsigned int g_win_height = 1134;

// 768x1134

/// * Included instead of using std::to_string because the latter
///   is missing on Android
template<typename T>
std::string ToString(T const &val)
{
    std::ostringstream oss;
    oss << val;
    return oss.str();
}

void Log(std::string s)
{
    #ifdef ENV_ANDROID_SDL
        __android_log_print(ANDROID_LOG_VERBOSE,"SDLTEST",s.c_str());
    #else
        std::cout << s << std::endl;
    #endif
}


// TEST
float g_input_direct_x=0;
float g_input_direct_y=0;

extern "C" {

    JNIEXPORT void JNICALL Java_org_libsdl_app_SDLActivity_onNativeTouchDirect(
            JNIEnv* env, jclass jcls, jfloat x, jfloat y)
    {
        (void)env;
        (void)jcls;
        g_input_direct_x = x;
        g_input_direct_y = y;
//        Log("Input: " + ToString(x) + "," + ToString(y));
    }

}

bool initSDL(SDL_Window * &window, SDL_GLContext &context)
{
    // init sdl
    SDL_Init(SDL_INIT_VIDEO);

    // set requested opengl context params
#ifdef ENV_ANDROID_SDL
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_ES);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,1);
#endif

    // create a window
    window = SDL_CreateWindow(
                "OpenGL",0,0,
                g_win_width,
                g_win_height,
                SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

    if(!window) {       
        Log("Error: Failed to create window" + std::string(SDL_GetError()));

        SDL_Quit();
        return false;
    }

    // get the opengl context
    context = SDL_GL_CreateContext(window);
    if(!context) {       
        Log("Error: Failed to create context" + std::string(SDL_GetError()));
        SDL_Quit();
        return false;
    }

    // load opengl functions (must be done after
    // getting a valid context)
#ifndef ENV_ANDROID_SDL
    if(!gladLoadGL()) {       
        Log("Error: Error loading OpenGL functions");
        SDL_GL_DeleteContext(context);
        SDL_Quit();
        return false;
    }
#endif

    if(SDL_GL_SetSwapInterval(1) < 0) {
        Log("Warn: Failed to set vsync: " + std::string(SDL_GetError()));
    }

    return true;
}

#ifndef ENV_ANDROID_SDL
static const std::string g_vsh_source =
        "#version 120\n"
        "\n"
        "// varyings\n"
        "varying vec4 v_v4_color;\n"
        "\n"
        "// attributes\n"
        "attribute vec4 a_v4_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = a_v4_position;\n"
        "}\n";

static const std::string g_fsh_source =
        "#version 120\n"
        "\n"
        "// varyings\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
        "}\n";
#else
static const std::string g_vsh_source =
        "#version 100\n"
        "\n"
        "\n"
        "// attributes\n"
        "attribute vec4 a_v4_position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_Position = a_v4_position;\n"
        "}\n";

static const std::string g_fsh_source =
        "#version 100\n"
        "\n"
        "// varyings\n"
        "\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
        "}\n";

#endif

bool initGL(GLuint &prog_id,
            GLuint &vbo_id,
            GLint &attrib_loc_position)
{
    (void)vbo_id;

    // vertex shader
    auto vsh_id = glCreateShader(GL_VERTEX_SHADER);
    char const * vsh_source = g_vsh_source.c_str();
    glShaderSource(vsh_id,1,&vsh_source,nullptr);
    glCompileShader(vsh_id);

    GLint vsh_ok = GL_FALSE;
    glGetShaderiv(vsh_id,GL_COMPILE_STATUS,&vsh_ok);
    if(vsh_ok==GL_FALSE) {
        Log("Error: Failed to compile vertex shader");
        return false;
    }

    // fragment shader
    auto fsh_id = glCreateShader(GL_FRAGMENT_SHADER);
    char const * fsh_source = g_fsh_source.c_str();
    glShaderSource(fsh_id,1,&fsh_source,nullptr);
    glCompileShader(fsh_id);

    GLint fsh_ok = GL_FALSE;
    glGetShaderiv(fsh_id,GL_COMPILE_STATUS,&fsh_ok);
    if(fsh_ok==GL_FALSE) {
        Log("Error: Failed to compile fragment shader");
        return false;
    }

    // create shader program
    prog_id = glCreateProgram();
    glAttachShader(prog_id,vsh_id);
    glAttachShader(prog_id,fsh_id);
    glLinkProgram(prog_id);

    GLint link_ok = GL_FALSE;
    glGetProgramiv(prog_id,GL_LINK_STATUS,&link_ok);
    if(link_ok==GL_FALSE) {
        Log("Error: Failed to link shader");
        glDeleteProgram(prog_id);
        return false;
    }

    // get attribute locations
    attrib_loc_position = glGetAttribLocation(prog_id,"a_v4_position");

    // set gl clear color
    glClearColor(0,0,0,1);

    return true;
}

SDL_Window * window;
SDL_GLContext context;
GLuint prog_id;
GLuint vbo_id{0};
GLint attrib_loc_position;

float input_x{0.0f};
float input_y{0.0f};
float input_s{170.0f};

float rads=0.0f;
float disp_x=0.0f;
float disp_y=0.0f;

float ToNDCX(float x)
{
    x /= float(g_win_width);
    x = x*2.0f - 1.0f;
    return x;
}

float ToNDCY(float y)
{
    y /= float(g_win_height);
    y = (1.0f-y)*2.0f - 1.0f;
    return y;
}

void render(GLuint prog_id,
            GLuint vbo_id,
            GLint attrib_loc_position)
{
    if(vbo_id != 0)
    {
        glDeleteBuffers(1,&vbo_id);
    }

    // create buffers
//    float dx = g_input_direct_x;
//    float dy = g_input_direct_y;

    float dx = input_x;
    float dy = input_y;

    GLfloat list_vx[] = {
        ToNDCX(0+dx), ToNDCY(0+dy), 0.0, 1.0, // tl
        ToNDCX(0+dx), ToNDCY(input_s+dy), 0.0, 1.0, // bl
        ToNDCX(input_s+dx), ToNDCY(input_s+dy), 0.0, 1.0, // br

        ToNDCX(0+dx), ToNDCY(0+dy), 0.0, 1.0, // tl
        ToNDCX(input_s+dx), ToNDCY(input_s+dy), 0.0, 1.0, // br
        ToNDCX(input_s+dx), ToNDCY(0+dy), 0.0, 1.0 // tr
    };

//    GLfloat list_vx[] = {
//        ToNDCX(0+input_x+disp_x), ToNDCY(0+input_y+disp_y), 0.0, 1.0, // tl
//        ToNDCX(0+input_x+disp_x), ToNDCY(input_s+input_y+disp_y), 0.0, 1.0, // bl
//        ToNDCX(input_s+input_x+disp_x), ToNDCY(input_s+input_y+disp_y), 0.0, 1.0, // br

//        ToNDCX(0+input_x+disp_x), ToNDCY(0+input_y+disp_y), 0.0, 1.0, // tl
//        ToNDCX(input_s+input_x+disp_x), ToNDCY(input_s+input_y+disp_y), 0.0, 1.0, // br
//        ToNDCX(input_s+input_x+disp_x), ToNDCY(0+input_y+disp_y), 0.0, 1.0 // tr
//    };

    rads += (6.28f/300.0f)*1.25f;
    disp_x = g_win_width/2 + sin(rads)*200;
    disp_y = g_win_height/2 + sin(rads*0.5)*400;

    glGenBuffers(1,&vbo_id);
    glBindBuffer(GL_ARRAY_BUFFER,vbo_id);
    glBufferData(GL_ARRAY_BUFFER,
                 4*6*sizeof(GLfloat),
                 list_vx,
                 GL_STREAM_DRAW);

    // clear color buffer
    glViewport(0,0,g_win_width,g_win_height);
    glClear(GL_COLOR_BUFFER_BIT);

    // render tri
    glUseProgram(prog_id);
    glEnableVertexAttribArray(attrib_loc_position);

    glBindBuffer(GL_ARRAY_BUFFER,vbo_id);
    glVertexAttribPointer(attrib_loc_position,4,GL_FLOAT,GL_FALSE,0,nullptr);

    glDrawArrays(GL_TRIANGLES,0,6);

    glDisableVertexAttribArray(attrib_loc_position);

    SDL_GL_SwapWindow(window);
}

void cleanup(SDL_GLContext &context,
             GLuint prog_id,
             GLuint vbo_id)
{
    glUseProgram(0);
    glDeleteProgram(prog_id);
    glDeleteBuffers(1,&vbo_id);
    SDL_GL_DeleteContext(context);
    SDL_Quit();
}



int onResize(void* , SDL_Event* ev)
{
    SDL_Event& event = *ev;
    if(event.type == SDL_WINDOWEVENT) {
        if(event.window.event == SDL_WINDOWEVENT_RESIZED) {
            g_win_width = event.window.data1;
            g_win_height = event.window.data2;
            render(prog_id,vbo_id,attrib_loc_position);
            Log("Resized window " + ToString(g_win_width) + "," + ToString(g_win_height));
            return 0;
        }
    }

    return 1;
}

int main(int argc, char* argv[])
{
    (void)argc; (void)argv;
    Log("ENTRY POINT");

    bool init_ok =
            initSDL(window,context) &&
            initGL(prog_id,vbo_id,attrib_loc_position);

    if(!init_ok) {
        Log("Init Failed");
        return -1;
    }

    // SDL_AddEventWatch(onResize,nullptr);
    SDL_SetEventFilter(onResize,nullptr);

    // poll for events from the window
    SDL_Event event;
    bool keep_running=true;

    while(keep_running)
    {
        // drain all available events
        while(SDL_PollEvent(&event) != 0) {
            if(event.type == SDL_QUIT) {
                keep_running=false;
                break;
            }
            else if(event.type == SDL_WINDOWEVENT) {
                if(event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    g_win_width = event.window.data1;
                    g_win_height = event.window.data2;
                }
            }
            else if(event.type == SDL_MOUSEMOTION)
            {
                input_x = event.motion.x;
                input_y = event.motion.y;
                break;
            }
        }

        render(prog_id,vbo_id,attrib_loc_position);
    }

    cleanup(context,prog_id,vbo_id);
    return 0;
}
