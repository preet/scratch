#include <cstdlib>
#include <iostream>

#include <glad/glad.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

int main()
{
    // init sdl
    SDL_Init(SDL_INIT_VIDEO);

    // set requested opengl context params
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,1);

    // create a window
    SDL_Window * window = SDL_CreateWindow("OpenGL",100,100,800,480,SDL_WINDOW_OPENGL);

    // get the opengl context
    SDL_GLContext context = SDL_GL_CreateContext(window);

    // load opengl functions (must be done after
    // getting a valid context)
    if(!gladLoadGL()) {
        std::cout << "Error loading OpenGL functions!" << std::endl;
        SDL_GL_DeleteContext(context);
        SDL_Quit();
        return -1;
    }


    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;

    // poll for events from the window
    SDL_Event windowEvent;
    while(true) {
        if(SDL_PollEvent(&windowEvent)) {
            if(windowEvent.type == SDL_QUIT) {
                break;
            }
        }

        // update window framebuffer for double
        // buffered context (default)
        SDL_GL_SwapWindow(window);
    }


    SDL_GL_DeleteContext(context);
    SDL_Quit();
	return 0;
}
