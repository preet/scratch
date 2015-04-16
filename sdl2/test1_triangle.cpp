#include <cstdlib>
#include <iostream>

#include <glad/glad.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

// directly based on
// http://lazyfoo.net/tutorials/SDL/51_SDL_and_modern_opengl/index.php

unsigned int g_win_width = 800;
unsigned int g_win_height = 480;


bool initSDL(SDL_Window * &window, SDL_GLContext &context)
{
    // init sdl
    SDL_Init(SDL_INIT_VIDEO);

    // set requested opengl context params
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,1);

    // create a window
    window = SDL_CreateWindow(
                "OpenGL",100,100,800,480,SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

    if(!window) {
        std::cout << "Error: Failed to create window: "
                  << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    // get the opengl context
    context = SDL_GL_CreateContext(window);
    if(!context) {
        std::cout << "Error: Failed to create context: "
                  << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    // load opengl functions (must be done after
    // getting a valid context)
    if(!gladLoadGL()) {
        std::cout << "Error loading OpenGL functions!" << std::endl;
        SDL_GL_DeleteContext(context);
        SDL_Quit();
        return false;
    }

    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;

    // set vsync
    if(SDL_GL_SetSwapInterval(1) < 0) {
        std::cout << "Warn: Failed to set vsync: "
                  << SDL_GetError() << std::endl;
    }

    return true;
}

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
        "    gl_FragColor = vec4(1.0, 0.5, 0.0, 1.0);\n"
        "}\n";

bool initGL(GLuint &prog_id,
            GLuint &vbo_id,
            GLint &attrib_loc_position)
{
    // vertex shader
    auto vsh_id = glCreateShader(GL_VERTEX_SHADER);
    char const * vsh_source = g_vsh_source.c_str();
    glShaderSource(vsh_id,1,&vsh_source,nullptr);
    glCompileShader(vsh_id);

    GLint vsh_ok = GL_FALSE;
    glGetShaderiv(vsh_id,GL_COMPILE_STATUS,&vsh_ok);
    if(vsh_ok==GL_FALSE) {
        std::cout << "Failed to compile vertex shader" << std::endl;
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
        std::cout << "Failed to compile fragment shader" << std::endl;
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
        std::cout << "Failed to link shader" << std::endl;
        glDeleteProgram(prog_id);
        return false;
    }

    // get attribute locations
    attrib_loc_position = glGetAttribLocation(prog_id,"a_v4_position");

    // create the vertex buffer
    GLfloat list_vx[] = {
        -1.0, -1.0, 0.0, 1.0,
         1.0, -1.0, 0.0, 1.0,
        -1.0,  1.0, 0.0, 1.0
    };

    glGenBuffers(1,&vbo_id);
    glBindBuffer(GL_ARRAY_BUFFER,vbo_id);
    glBufferData(GL_ARRAY_BUFFER,
                 4*4*sizeof(GLfloat),
                 list_vx,
                 GL_STATIC_DRAW);

    // set gl clear color
    glClearColor(0.1, 0.1, 0.1, 1.0);

    return true;
}

void render(GLuint prog_id,
            GLuint vbo_id,
            GLint attrib_loc_position)
{
    // clear color buffer
    glViewport(0,0,g_win_width,g_win_height);
    glClear(GL_COLOR_BUFFER_BIT);


    // render tri
    glUseProgram(prog_id);
    glEnableVertexAttribArray(attrib_loc_position);

    glBindBuffer(GL_ARRAY_BUFFER,vbo_id);
    glVertexAttribPointer(attrib_loc_position,4,GL_FLOAT,GL_FALSE,0,nullptr);

    glDrawArrays(GL_TRIANGLES,0,3);

    glDisableVertexAttribArray(attrib_loc_position);
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

int main()
{
    SDL_Window * window;
    SDL_GLContext context;
    GLuint prog_id;
    GLuint vbo_id;
    GLint attrib_loc_position;

    bool init_ok =
            initSDL(window,context) &&
            initGL(prog_id,vbo_id,attrib_loc_position);

    if(!init_ok) {
        return -1;
    }

    // poll for events from the window
    SDL_Event event;
    bool keep_running=true;
    while(keep_running) {
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
        }

        render(prog_id,vbo_id,attrib_loc_position);

        // update window framebuffer for double
        // buffered context (default)
        SDL_GL_SwapWindow(window);
    }

    cleanup(context,prog_id,vbo_id);
	return 0;
}
