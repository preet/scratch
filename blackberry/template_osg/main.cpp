/*
 * Copyright (c) 2011-2012 Research In Motion Limited.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <assert.h>
#include <screen/screen.h>
#include <bps/navigator.h>
#include <bps/screen.h>
#include <bps/bps.h>
#include <bps/event.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "bbutil.h"

#include <osg/ref_ptr>
#include <osgDB/ReadFile>
#include <osg/AnimationPath>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/CullFace>
#include <osgUtil/SmoothingVisitor>

#include "openctm/openctm.h"

static screen_context_t screen_cxt;

static const char gVertexShader[] =
    "varying vec4 color;                                                    \n"
    "const vec3 lightPos      =vec3(25.0, 25.0, 50.0);                      \n"
    "const vec4 cessnaColor   =vec4(0.8, 0.8, 0.8, 1.0);                    \n"
    "const vec4 lightAmbient  =vec4(0.1, 0.1, 0.1, 1.0);                    \n"
    "const vec4 lightDiffuse  =vec4(0.4, 0.4, 0.4, 1.0);                    \n"
    "const vec4 lightSpecular =vec4(0.8, 0.8, 0.8, 1.0);                    \n"
    "void DirectionalLight(in vec3 normal,                                  \n"
    "                      in vec3 ecPos,                                   \n"
    "                      inout vec4 ambient,                              \n"
    "                      inout vec4 diffuse,                              \n"
    "                      inout vec4 specular)                             \n"
    "{                                                                      \n"
    "     float nDotVP;                                                     \n"
    "     vec3 L = normalize(gl_ModelViewMatrix*vec4(lightPos, 0.0)).xyz;   \n"
    "     nDotVP = max(0.0, dot(normal, L));                                \n"
    "                                                                       \n"
    "     if (nDotVP > 0.0) {                                               \n"
    "       vec3 E = normalize(-ecPos);                                     \n"
    "       vec3 R = normalize(reflect( L, normal ));                       \n"
    "       specular = pow(max(dot(R, E), 0.0), 16.0) * lightSpecular;      \n"
    "     }                                                                 \n"
    "     ambient  = lightAmbient;                                          \n"
    "     diffuse  = lightDiffuse * nDotVP;                                 \n"
    "}                                                                      \n"
    "void main() {                                                          \n"
    "    vec4 ambiCol = vec4(0.0);                                          \n"
    "    vec4 diffCol = vec4(0.0);                                          \n"
    "    vec4 specCol = vec4(0.0);                                          \n"
    "    gl_Position   = gl_ModelViewProjectionMatrix * gl_Vertex;          \n"
    "    vec3 normal   = normalize(gl_NormalMatrix * gl_Normal);            \n"
    "    vec4 ecPos    = gl_ModelViewMatrix * gl_Vertex;                    \n"
    "    DirectionalLight(normal, ecPos.xyz, ambiCol, diffCol, specCol);    \n"
    "    color = cessnaColor * (ambiCol + diffCol + specCol);               \n"
    "}                                                                      \n";

static const char gFragmentShader[] =
    "precision mediump float;                  \n"
    "varying mediump vec4 color;               \n"
    "void main() {                             \n"
    "  gl_FragColor = color;                   \n"
    "}                                         \n";


void handleScreenEvent(bps_event_t *event) {
    screen_event_t screen_event = screen_event_get_event(event);

    int screen_val;
    screen_get_event_property_iv(screen_event, SCREEN_PROPERTY_TYPE, &screen_val);

    switch (screen_val) {
    case SCREEN_EVENT_MTOUCH_TOUCH:
    case SCREEN_EVENT_MTOUCH_MOVE:
    case SCREEN_EVENT_MTOUCH_RELEASE:
        break;
    }
}

int main(int argc, char *argv[]) {
    int rc;
    int exit_application = 0;

    //Create a screen context that will be used to create an EGL surface to to receive libscreen events
    screen_create_context(&screen_cxt, 0);

    //Initialize BPS library
    bps_initialize();

    //Use utility code to initialize EGL for rendering with GL ES 2.0
    if (EXIT_SUCCESS != bbutil_init_egl(screen_cxt)) {
        fprintf(stderr, "bbutil_init_egl failed\n");
        bbutil_terminate();
        screen_destroy_context(screen_cxt);
        return 0;
    }

    //Initialize application logic
    osg::setNotifyLevel(osg::DEBUG_INFO);

    // node: interesting geometry
    CTMcontext cContext;
    CTMuint vertCount,triCount;
    CTMuint const * indices;
    CTMfloat const * vertices;
    CTMfloat const * normals;

    cContext = ctmNewContext(CTM_IMPORT);
    ctmLoad(cContext,"app/native/models/cow.ctm");
    if(ctmGetError(cContext) == CTM_NONE)
    {
        // access the mesh data
        vertCount = ctmGetInteger(cContext, CTM_VERTEX_COUNT);
        vertices = ctmGetFloatArray(cContext, CTM_VERTICES);
        triCount = ctmGetInteger(cContext, CTM_TRIANGLE_COUNT);
        indices = ctmGetIntegerArray(cContext, CTM_INDICES);

        std::cout << "# Mesh has " << vertCount << " vertices\n";
        std::cout << "# Mesh has " << triCount << " triangles\n";
    }
    else
    {
        std::cout << "Error Reading CTM File!" << std::endl;
        return -1;
    }

    // build up openscenegraph geometry
    osg::ref_ptr<osg::Vec3Array> listVxArray = new osg::Vec3Array(vertCount);
    unsigned int vxIdx=0;
    for(int i=0; i < listVxArray->size(); i++)   {
        osg::Vec3 vertex;
        vertex.x() = vertices[vxIdx]; vxIdx++;
        vertex.y() = vertices[vxIdx]; vxIdx++;
        vertex.z() = vertices[vxIdx]; vxIdx++;
        listVxArray->at(i) = vertex;
    }

    osg::ref_ptr<osg::DrawElementsUInt> listIdxs =
            new osg::DrawElementsUInt(GL_TRIANGLES,triCount*3);
    for(int i=0; i < listIdxs->size(); i++)   {
        listIdxs->at(i) = indices[i];
    }

    osg::ref_ptr<osg::Geometry> geomMesh = new osg::Geometry;
    geomMesh->setVertexArray(listVxArray.get());
    geomMesh->addPrimitiveSet(listIdxs.get());
    osgUtil::SmoothingVisitor::smooth(*geomMesh);

    osg::ref_ptr<osg::Geode> geodeMesh = new osg::Geode;
    geodeMesh->addDrawable(geomMesh.get());

    osg::ref_ptr<osg::Group> groupRoot = new osg::Group;
    groupRoot->addChild(geodeMesh.get());

    // free ctm memory
    ctmFreeContext(cContext);

    // shader
    osg::StateSet *ss = geodeMesh->getOrCreateStateSet();
    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->setName( "simpleshader" );
    program->addShader( new osg::Shader( osg::Shader::VERTEX, gVertexShader ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, gFragmentShader ) );
    ss->setAttributeAndModes(program, osg::StateAttribute::ON);
//    ss->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
//    ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::OFF);
//    ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON);

    // rotate that cube
    osg::ref_ptr<osg::MatrixTransform> nodeSpin = new osg::MatrixTransform;
    nodeSpin->addChild(geodeMesh.get());
    nodeSpin->addUpdateCallback(new osg::AnimationPathCallback(osg::Vec3(0,0,0),
                                                               osg::Y_AXIS,
                                                               osg::inDegrees(45.0f)));
    // node: root
    osg::ref_ptr<osg::Group> nodeRoot = new osg::Group;
    nodeRoot->addChild(nodeSpin.get());

    // center point
    osg::BoundingBox modelBounds = geodeMesh->getBoundingBox();

    // viewer
    osgViewer::Viewer myViewer;
    myViewer.setSceneData(nodeRoot.get());
    myViewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3((modelBounds.xMax()-modelBounds.xMin())*2,
                                                          (modelBounds.yMax()-modelBounds.yMin())*2,
                                                          (modelBounds.zMax()-modelBounds.zMin())*2),
                                                modelBounds.center(),
                                                osg::Vec3(0,1,0));

    // graphics window embedded
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> myWindow =
            new osgViewer::GraphicsWindowEmbedded(0,0,1024,600);
    myWindow->getState()->setUseModelViewAndProjectionUniforms(true);
    myWindow->getState()->setUseVertexAttributeAliasing(true);

    // setup viewer
    myViewer.getCamera()->setViewport(new osg::Viewport(0,0,1024,600));
    myViewer.getCamera()->setGraphicsContext(myWindow.get());
    myViewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

    //Signal BPS library that navigator and screen events will be requested
    if (BPS_SUCCESS != screen_request_events(screen_cxt)) {
        fprintf(stderr, "screen_request_events failed\n");
        bbutil_terminate();
        screen_destroy_context(screen_cxt);
        bps_shutdown();
        return 0;
    }

    if (BPS_SUCCESS != navigator_request_events(0)) {
        fprintf(stderr, "navigator_request_events failed\n");
        bbutil_terminate();
        screen_destroy_context(screen_cxt);
        bps_shutdown();
        return 0;
    }

    //Signal BPS library that navigator orientation is not to be locked
    if (BPS_SUCCESS != navigator_rotation_lock(false)) {
        fprintf(stderr, "navigator_rotation_lock failed\n");
        bbutil_terminate();
        screen_destroy_context(screen_cxt);
        bps_shutdown();
        return 0;
    }

    while (!exit_application) {
        //Request and process all available BPS events
        bps_event_t *event = NULL;

        for(;;) {
            rc = bps_get_event(&event, 0);
            assert(rc == BPS_SUCCESS);

            if (event) {
                int domain = bps_event_get_domain(event);

                if (domain == screen_get_domain()) {
                    handleScreenEvent(event);
                } else if ((domain == navigator_get_domain())
                        && (NAVIGATOR_EXIT == bps_event_get_code(event))) {
                    exit_application = 1;
                }
            } else {
                break;
            }
        }
        myViewer.frame();
        bbutil_swap();
    }

    //Stop requesting events from libscreen
    screen_stop_events(screen_cxt);

    //Shut down BPS library for this process
    bps_shutdown();

    //Use utility code to terminate EGL setup
    bbutil_terminate();

    //Destroy libscreen context
    screen_destroy_context(screen_cxt);
    return 0;
}
