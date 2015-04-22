/*-----------------------------------------------------------------------
  Copyright (c) 2014, NVIDIA. All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Neither the name of its contributors may be used to endorse 
     or promote products derived from this software without specific
     prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------*/
/* Contact ckubisch@nvidia.com (Christoph Kubisch) for feedback */

#define DEBUG_FILTER     1

#include <GL/glew.h>
#include <nv_helpers/anttweakbar.hpp>
#include <nv_helpers_gl/WindowProfiler.hpp>
#include <nv_math/nv_math_glsltypes.h>

#include <nv_helpers_gl/error.hpp>
#include <nv_helpers_gl/programmanager.hpp>
#include <nv_helpers/geometry.hpp>
#include <nv_helpers/misc.hpp>
#include <nv_helpers_gl/glresources.hpp>
#include <nv_helpers/cameracontrol.hpp>

#include <noise/MersenneTwister.h>

// optimizes blur, by storing depth along with ssao calculation
// avoids accessing two different textures
#define USE_AO_SPECIALBLUR          1

// optimizes the cache-aware technique by rendering all temporary layers at once
// instead of individually
#define USE_AO_LAYERED_SINGLEPASS   1

using namespace nv_helpers;
using namespace nv_helpers_gl;
using namespace nv_math;
#include "common.h"

namespace ssao
{
#if 1
  int const SAMPLE_SIZE_WIDTH(1280);
  int const SAMPLE_SIZE_HEIGHT(720);
#else
  int const SAMPLE_SIZE_WIDTH(1920);
  int const SAMPLE_SIZE_HEIGHT(1080);
#endif
  int const SAMPLE_MAJOR_VERSION(4);
  int const SAMPLE_MINOR_VERSION(3);

  static const int  NUM_MRT = 8;
  static const int  HBAO_RANDOM_SIZE = AO_RANDOMTEX_SIZE;
  static const int  HBAO_RANDOM_ELEMENTS = HBAO_RANDOM_SIZE*HBAO_RANDOM_SIZE;
  static const int  MAX_SAMPLES = 8;

  static const int        grid = 32;
  static const float      globalscale = 16.0f;

  class Sample : public nv_helpers_gl::WindowProfiler
  {
    ProgramManager progManager;

    enum AlgorithmType {
      ALGORITHM_NONE,
      ALGORITHM_HBAO_CACHEAWARE,
      ALGORITHM_HBAO_CLASSIC,
      NUM_ALGORITHMS,
    };

    struct {
      ProgramManager::ProgramID
        draw_scene,
        depth_linearize,
        depth_linearize_msaa,
        viewnormal,
        bilateralblur,
        displaytex,

        hbao_calc,
        hbao_calc_blur,
        hbao_blur,
        hbao_blur2,

        hbao2_deinterleave,
        hbao2_calc,
        hbao2_calc_blur,
        hbao2_reinterleave,
        hbao2_reinterleave_blur;

    } programs;

    struct {
      ResourceGLuint
        scene,
        depthlinear,
        viewnormal,
        hbao_calc,
        hbao2_deinterleave,
        hbao2_calc;
    } fbos;

    struct {
      ResourceGLuint  
        scene_vbo,
        scene_ibo,
        scene_ubo,
        hbao_ubo;
    } buffers;

    struct {
      ResourceGLuint
        scene_color,
        scene_depthstencil,
        scene_depthlinear,
        scene_viewnormal,
        hbao_result,
        hbao_blur,
        hbao_random,
        hbao_randomview[MAX_SAMPLES],
        hbao2_deptharray,
        hbao2_depthview[HBAO_RANDOM_ELEMENTS],
        hbao2_resultarray;
    } textures;

    struct Vertex {

      Vertex(const geometry::Vertex& vertex){
        position  = vertex.position;
        normal    = vertex.normal;
        color     = nv_math::vec4(1.0f);
      }

      nv_math::vec4   position;
      nv_math::vec4   normal;
      nv_math::vec4   color;
    };


    struct Tweak {
      Tweak() 

        : algorithm(ALGORITHM_HBAO_CACHEAWARE)
        , samples(1)
        , intensity(1.5f)
        , radius(2.f)
        , bias(0.1f)
        , blur(1)
        , blurSharpness(40.0f)
      {}

      int             samples;
      AlgorithmType   algorithm;
      float           intensity;
      float           bias;
      float           radius;
      int             blur;
      float           blurSharpness;
    };

    Tweak      tweak;
    Tweak      tweakLast;
    uint       sceneTriangleIndices;
    uint       sceneObjects;

    vec4f      hbaoRandom[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES];

    struct Projection {
      float nearplane;
      float farplane;
      float fov;
      mat4  matrix;

      Projection()
        : nearplane(0.1f)
        , farplane(100.0f)
        , fov((45.f))
      {

      }

      void update(int width, int height){
        matrix =  nv_math::perspective(fov, float(width)/float(height), nearplane, farplane);
      }
    };

    SceneData  sceneUbo;
    HBAOData   hbaoUbo;

    bool begin();
    void think(double time);
    void resize(int width, int height);

    void prepareHbaoData(const Projection& projection, int width, int height);

    void drawLinearDepth(const Projection& projection, int width, int height, int sampleIdx);
    void drawHbaoBlur(const Projection& projection, int width, int height, int sampleIdx);
    void drawHbaoClassic(const Projection& projection, int width, int height, int sampleIdx);
    void drawHbaoCacheAware(const Projection& projection, int width, int height, int sampleIdx);

    bool initProgram();
    bool initScene();
    bool initMisc();
    bool initFramebuffers(int width, int height, int samples);

    CameraControl m_control;

    void end() {
      TwTerminate();
    }
    // return true to prevent m_window updates
    bool mouse_pos    (int x, int y) {
      return !!TwEventMousePosGLFW(x,y); 
    }
    bool mouse_button (int button, int action) {
      return !!TwEventMouseButtonGLFW(button, action);
    }
    bool mouse_wheel  (int wheel) {
      return !!TwEventMouseWheelGLFW(wheel); 
    }
    bool key_button   (int button, int action, int mods) {
      return handleTwKeyPressed(button,action,mods);
    }
    
  };

  bool Sample::initProgram()
  {
    bool validated(true);
    progManager.addDirectory( std::string(PROJECT_NAME));
    progManager.addDirectory( sysExePath() + std::string(PROJECT_RELDIRECTORY));
    progManager.addDirectory( std::string(PROJECT_ABSDIRECTORY));

    progManager.registerInclude("common.h", "common.h");

    progManager.m_prepend = ProgramManager::format("#define AO_LAYERED %d\n", USE_AO_LAYERED_SINGLEPASS ? 1 : 0);

    programs.draw_scene = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "scene.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "scene.frag.glsl"));

    programs.bilateralblur = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "bilateralblur.frag.glsl"));

    programs.depth_linearize = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define DEPTHLINEARIZE_MSAA 0\n", "depthlinearize.frag.glsl"));

    programs.depth_linearize_msaa = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define DEPTHLINEARIZE_MSAA 1\n", "depthlinearize.frag.glsl"));

    programs.viewnormal = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "viewnormal.frag.glsl"));

    programs.displaytex = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "displaytex.frag.glsl"));

    programs.hbao_calc = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 0\n", "hbao.frag.glsl"));

    programs.hbao_calc_blur = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao_blur = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR_PRESENT 0\n","hbao_blur.frag.glsl"));

    programs.hbao_blur2 = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR_PRESENT 1\n","hbao_blur.frag.glsl"));

    programs.hbao2_calc = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 0\n", "hbao.frag.glsl"));

    programs.hbao2_calc_blur = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao2_deinterleave = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "hbao_deinterleave.frag.glsl"));

    programs.hbao2_reinterleave = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR 0\n","hbao_reinterleave.frag.glsl"));

    programs.hbao2_reinterleave_blur = progManager.createProgram(
      ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR 1\n","hbao_reinterleave.frag.glsl"));

    validated = progManager.areProgramsValid();

    return validated;
  }

  bool Sample::initMisc()
  {
    MTRand rng;

    float numDir = 8; // keep in sync to glsl

    rng.seed((unsigned)0);

    signed short hbaoRandomShort[HBAO_RANDOM_ELEMENTS*MAX_SAMPLES*4];

    for(int i=0; i<HBAO_RANDOM_ELEMENTS*MAX_SAMPLES; i++)
    {
      float Rand1 = rng.randExc();
      float Rand2 = rng.randExc();

      // Use random rotation angles in [0,2PI/NUM_DIRECTIONS)
      float Angle = 2.f * nv_pi * Rand1 / numDir;
      hbaoRandom[i].x = cosf(Angle);
      hbaoRandom[i].y = sinf(Angle);
      hbaoRandom[i].z = Rand2;
      hbaoRandom[i].w = 0;
#define SCALE ((1<<15))
      hbaoRandomShort[i*4+0] = (signed short)(SCALE*hbaoRandom[i].x);
      hbaoRandomShort[i*4+1] = (signed short)(SCALE*hbaoRandom[i].y);
      hbaoRandomShort[i*4+2] = (signed short)(SCALE*hbaoRandom[i].z);
      hbaoRandomShort[i*4+3] = (signed short)(SCALE*hbaoRandom[i].w);
#undef SCALE
    }

    newTexture(textures.hbao_random);
    glBindTexture(GL_TEXTURE_2D_ARRAY,textures.hbao_random);
    glTexStorage3D (GL_TEXTURE_2D_ARRAY,1,GL_RGBA16_SNORM,HBAO_RANDOM_SIZE,HBAO_RANDOM_SIZE,MAX_SAMPLES);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY,0,0,0,0, HBAO_RANDOM_SIZE,HBAO_RANDOM_SIZE,MAX_SAMPLES,GL_RGBA,GL_SHORT,hbaoRandomShort);
    glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D_ARRAY,0);

    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      newTexture(textures.hbao_randomview[i]);
      glTextureView(textures.hbao_randomview[i], GL_TEXTURE_2D, textures.hbao_random, GL_RGBA16_SNORM, 0, 1, i, 1);
      glBindTexture(GL_TEXTURE_2D, textures.hbao_randomview[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glBindTexture(GL_TEXTURE_2D, 0);
    }

    newBuffer(buffers.hbao_ubo);
    glNamedBufferStorageEXT(buffers.hbao_ubo, sizeof(HBAOData), NULL, GL_DYNAMIC_STORAGE_BIT);

    return true;
  }

  bool Sample::initScene()
  {
    { // Scene Geometry
      geometry::Mesh<Vertex>  scene;
      const int LEVELS = 4;
      
      sceneObjects = 0;
      for (int i = 0; i < grid * grid; i++){

        vec4 color(frand(),frand(),frand(),1.0f);
        color *= 0.25f;
        color += 0.75f;

        vec2  posxy(i % grid, i / grid);
        
        float depth = sin(posxy.x*0.1f) * cos(posxy.y*0.1f) * 2.0f;


        for (int l = 0; l < LEVELS; l++){
          vec3  pos(posxy.x, posxy.y, depth);

          float scale = globalscale * 0.5f/float(grid);
          if (l != 0){
            scale *= powf(0.9f,float(l));
            scale *= frand()*0.5f + 0.5f;
          }

          vec3 size = vec3(scale);
          

          size.z *= frand()*1.0f+1.0f; 
          if (l != 0){
            size.z *= powf(0.7f,float(l));
          }

          pos -=  vec3( grid/2, grid/2, 0);
          pos /=  float(grid) / globalscale;

          depth += size.z;

          pos.z = depth;

          mat4  matrix    = nv_math::translation_mat4( pos) * nv_math::scale_mat4( size);

          uint  oldverts  = scene.getVerticesCount();
          uint  oldinds   = scene.getTriangleIndicesCount();

          geometry::Box<Vertex>::add(scene,matrix,2,2,2);

          for (uint v = oldverts; v < scene.getVerticesCount(); v++){
            scene.m_vertices[v].color = color;
          }

          depth += size.z;
        }

        sceneObjects++;
      }

      sceneTriangleIndices = scene.getTriangleIndicesCount();

      newBuffer(buffers.scene_ibo);
      glNamedBufferStorageEXT(buffers.scene_ibo, scene.getTriangleIndicesSize(), &scene.m_indicesTriangles[0], 0);

      newBuffer(buffers.scene_vbo);
      glBindBuffer(GL_ARRAY_BUFFER, buffers.scene_vbo);
      glNamedBufferStorageEXT(buffers.scene_vbo, scene.getVerticesSize(), &scene.m_vertices[0], 0);

      glVertexAttribFormat(VERTEX_COLOR,  4, GL_FLOAT, GL_FALSE,  offsetof(Vertex,color));
      glVertexAttribBinding(VERTEX_COLOR, 0);

      glVertexAttribFormat(VERTEX_POS,    3, GL_FLOAT, GL_FALSE,  offsetof(Vertex,position));
      glVertexAttribFormat(VERTEX_NORMAL, 3, GL_FLOAT, GL_FALSE,  offsetof(Vertex,normal));
      glVertexAttribBinding(VERTEX_POS,   0);
      glVertexAttribBinding(VERTEX_NORMAL,0);
    }

    { // Scene UBO
      newBuffer(buffers.scene_ubo);
      glNamedBufferStorageEXT(buffers.scene_ubo, sizeof(SceneData), NULL, GL_DYNAMIC_STORAGE_BIT);
    }

    return true;
  }

  bool Sample::initFramebuffers(int width, int height, int samples)
  {

    if (samples > 1){
      newTexture(textures.scene_color);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, textures.scene_color);
      glTexStorage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_RGBA8, width, height, GL_FALSE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, 0);

      newTexture(textures.scene_depthstencil);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, textures.scene_depthstencil);
      glTexStorage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_DEPTH24_STENCIL8, width, height, GL_FALSE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, 0);
    }
    else
    {
      newTexture(textures.scene_color);
      glBindTexture (GL_TEXTURE_2D, textures.scene_color);
      glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
      glBindTexture (GL_TEXTURE_2D, 0);

      newTexture(textures.scene_depthstencil);
      glBindTexture (GL_TEXTURE_2D, textures.scene_depthstencil);
      glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, width, height);
      glBindTexture (GL_TEXTURE_2D, 0);
    }

    newFramebuffer(fbos.scene);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.scene);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,        textures.scene_color, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, textures.scene_depthstencil, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    newTexture(textures.scene_depthlinear);
    glBindTexture (GL_TEXTURE_2D, textures.scene_depthlinear);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D, 0);

    newFramebuffer(fbos.depthlinear);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.depthlinear);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,  textures.scene_depthlinear, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    newTexture(textures.scene_viewnormal);
    glBindTexture (GL_TEXTURE_2D, textures.scene_viewnormal);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D, 0);

    newFramebuffer(fbos.viewnormal);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.viewnormal);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,  textures.scene_viewnormal, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // hbao

#if USE_AO_SPECIALBLUR
    GLenum formatAO = GL_RG16F;
    GLint swizzle[4] = {GL_RED,GL_GREEN,GL_ZERO,GL_ZERO};
#else
    GLenum formatAO = GL_R8;
    GLint swizzle[4] = {GL_RED,GL_RED,GL_RED,GL_RED};
#endif
    
    newTexture(textures.hbao_result);
    glBindTexture (GL_TEXTURE_2D, textures.hbao_result);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture (GL_TEXTURE_2D, 0);

    newTexture(textures.hbao_blur);
    glBindTexture (GL_TEXTURE_2D, textures.hbao_blur);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture (GL_TEXTURE_2D, 0);

    newFramebuffer(fbos.hbao_calc);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.hbao_calc);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textures.hbao_result, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, textures.hbao_blur, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // interleaved hbao

    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    newTexture(textures.hbao2_deptharray);
    glBindTexture (GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32F, quarterWidth, quarterHeight, HBAO_RANDOM_ELEMENTS);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D_ARRAY, 0);

    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
      newTexture(textures.hbao2_depthview[i]);
      glTextureView(textures.hbao2_depthview[i], GL_TEXTURE_2D, textures.hbao2_deptharray, GL_R32F, 0, 1, i, 1);
      glBindTexture(GL_TEXTURE_2D, textures.hbao2_depthview[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glBindTexture(GL_TEXTURE_2D, 0);
    }


    newTexture(textures.hbao2_resultarray);
    glBindTexture (GL_TEXTURE_2D_ARRAY, textures.hbao2_resultarray);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, formatAO, quarterWidth, quarterHeight, HBAO_RANDOM_ELEMENTS);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D_ARRAY, 0);


    GLenum drawbuffers[NUM_MRT];
    for (int layer = 0; layer < NUM_MRT; layer++){
      drawbuffers[layer] = GL_COLOR_ATTACHMENT0 + layer;
    }

    newFramebuffer(fbos.hbao2_deinterleave);
    glBindFramebuffer(GL_FRAMEBUFFER,fbos.hbao2_deinterleave);
    glDrawBuffers(NUM_MRT,drawbuffers);
    glBindFramebuffer(GL_FRAMEBUFFER,0);

    newFramebuffer(fbos.hbao2_calc);
    glBindFramebuffer(GL_FRAMEBUFFER,fbos.hbao2_calc);
#if USE_AO_LAYERED_SINGLEPASS
    // this fbo will not have any attachments and therefore requires rasterizer to be configured
    // through default parameters
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH,  quarterWidth);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, quarterHeight);
#endif
    glBindFramebuffer(GL_FRAMEBUFFER,0);

    return true;
  }


  bool Sample::begin()
  {
    TwInit(TW_OPENGL_CORE,NULL);
    TwWindowSize(m_window.m_viewsize[0],m_window.m_viewsize[1]);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    bool validated(true);

    GLuint defaultVAO;
    glGenVertexArrays(1, &defaultVAO);
    glBindVertexArray(defaultVAO);

    validated = validated && initProgram();
    validated = validated && initMisc();
    validated = validated && initScene();
    validated = validated && initFramebuffers(m_window.m_viewsize[0],m_window.m_viewsize[1],tweak.samples);

    TwBar *bar = TwNewBar("mainbar");
    TwDefine(" GLOBAL contained=true help='OpenGL samples.\nCopyright NVIDIA Corporation 2013-2014' ");
    TwDefine(" mainbar position='0 0' size='300 150' color='0 0 0' alpha=128 valueswidth=120 ");
    TwDefine((std::string(" mainbar label='") + PROJECT_NAME + "'").c_str());

    TwEnumVal enumVals[] = {
      {ALGORITHM_NONE,"none"},
      {ALGORITHM_HBAO_CACHEAWARE,"hbao cache-aware"},
      {ALGORITHM_HBAO_CLASSIC,"hbao classic"},
    };
    TwType algorithmType = TwDefineEnum("algorithm", enumVals, sizeof(enumVals)/sizeof(enumVals[0]));

    TwEnumVal enumSampleVals[] = {
      {1,"none"},
      {2,"2x"},
      {4,"4x"},
      {8,"8x"},
    };
    TwType samplesType = TwDefineEnum("samples", enumSampleVals, sizeof(enumSampleVals)/sizeof(enumSampleVals[0]));

    TwAddVarRW(bar, "samples",  samplesType, &tweak.samples, " label='msaa' ");
    TwAddVarRW(bar, "algorithm",  algorithmType, &tweak.algorithm, " label='ssao algorithm' ");
    TwAddVarRW(bar, "radius",  TW_TYPE_FLOAT, &tweak.radius, " label='radius' step=0.1 min=0 precision=2 ");
    TwAddVarRW(bar, "intensity",  TW_TYPE_FLOAT, &tweak.intensity, " label='intensity' min=0 step=0.1 ");
    TwAddVarRW(bar, "bias",  TW_TYPE_FLOAT, &tweak.bias, " label='bias' min=0 step=0.1 max=0.1");
    TwAddVarRW(bar, "bluractive",  TW_TYPE_BOOL32, &tweak.blur, " label='blur active' ");
    TwAddVarRW(bar, "blursharpness",  TW_TYPE_FLOAT, &tweak.blurSharpness, " label='blur sharpness' min=0 ");

    m_control.m_sceneOrbit = vec3(0.0f);
    m_control.m_sceneDimension = float(globalscale);
    m_control.m_viewMatrix = nv_math::look_at(m_control.m_sceneOrbit - (vec3(0.4f,-0.35f,-0.6f)*m_control.m_sceneDimension*0.5f), m_control.m_sceneOrbit, vec3(0,1,0));
    
    return validated;
  }

  void Sample::prepareHbaoData(const Projection& projection, int width, int height)
  {
    // projection
    const float* P = projection.matrix.get_value();

    float projInfoPerspective[] = {
      2.0f / (P[4*0+0]),       // (x) * (R - L)/N
      2.0f / (P[4*1+1]),       // (y) * (T - B)/N
      -( 1.0f - P[4*2+0]) / P[4*0+0], // L/N
      -( 1.0f + P[4*2+1]) / P[4*1+1], // B/N
    };

    float projInfoOrtho[] = {
      2.0f / ( P[4*0+0]),      // ((x)  * R - L)
      2.0f / ( P[4*1+1]),      // ((y) * T - B)
      -( 1.0f + P[4*3+0]) / P[4*0+0], // L
      -( 1.0f - P[4*3+1]) / P[4*1+1], // B
    };

    int useOrtho = 0;
    hbaoUbo.projOrtho = useOrtho;
    hbaoUbo.projInfo  = useOrtho ? projInfoOrtho : projInfoPerspective;

    float projScale;
    if (useOrtho){
      projScale = float(height) / ( 8.0f /* FIXME need proper values for ortho */ );
    }
    else {
      projScale = float(height) / (tanf( projection.fov * 0.5f) * 2.0f);
    }

    // radius
    float meters2viewspace = 1.0f;
    float R = tweak.radius * meters2viewspace;
    hbaoUbo.R2 = R * R;
    hbaoUbo.NegInvR2 = -1.0f / hbaoUbo.R2;
    hbaoUbo.RadiusToScreen = R * 0.5f * projScale;

    // ao
    hbaoUbo.PowExponent = std::max(tweak.intensity,0.0f);
    hbaoUbo.NDotVBias = std::min(std::max(0.0f, tweak.bias),1.0f);
    hbaoUbo.AOMultiplier = 1.0f / (1.0f - hbaoUbo.NDotVBias);

    // resolution
    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    hbaoUbo.InvQuarterResolution = vec2(1.0f/float(quarterWidth),1.0f/float(quarterHeight));
    hbaoUbo.InvFullResolution = vec2(1.0f/float(width),1.0f/float(height));

#if USE_AO_LAYERED_SINGLEPASS
    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
      hbaoUbo.float2Offsets[i] = vec2(float(i % 4) + 0.5f, float(i / 4) + 0.5f);
      hbaoUbo.jitters[i] = hbaoRandom[i];
    }
#endif
  }

  void Sample::drawLinearDepth(const Projection& projection, int width, int height, int sampleIdx)
  {
    NV_PROFILE_SECTION("linearize");
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);

    if (tweak.samples > 1){
      glUseProgram(progManager.get(programs.depth_linearize_msaa));
      glUniform4f(0,projection.nearplane * projection.farplane, projection.nearplane-projection.farplane, projection.farplane, 1.0f);
      glUniform1i(1,sampleIdx);

      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, textures.scene_depthstencil);
      glDrawArrays(GL_TRIANGLES,0,3);
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, 0);
    }
    else{
      glUseProgram(progManager.get(programs.depth_linearize));
      glUniform4f(0,projection.nearplane * projection.farplane, projection.nearplane-projection.farplane, projection.farplane, 1.0f);

      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthstencil);
      glDrawArrays(GL_TRIANGLES,0,3);
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    }
  }

  void Sample::drawHbaoBlur(const Projection& projection, int width, int height, int sampleIdx)
  {
    NV_PROFILE_SECTION("ssaoblur");

    float meters2viewspace = 1.0f;

    glUseProgram(progManager.get(USE_AO_SPECIALBLUR ? programs.hbao_blur : programs.bilateralblur));
    glBindMultiTextureEXT(GL_TEXTURE1, GL_TEXTURE_2D, textures.scene_depthlinear);

    glUniform1f(0,tweak.blurSharpness/meters2viewspace);

    glDrawBuffer(GL_COLOR_ATTACHMENT1);

    glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_result);
    glUniform2f(1,1.0f/float(width),0);
    glDrawArrays(GL_TRIANGLES,0,3);

    // final output to main fbo
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ZERO,GL_SRC_COLOR);
    if (tweak.samples > 1){
      glEnable(GL_SAMPLE_MASK);
      glSampleMaski(0, 1<<sampleIdx);
    }

#if USE_AO_SPECIALBLUR
    glUseProgram(progManager.get(programs.hbao_blur2));
    glUniform1f(0,tweak.blurSharpness/meters2viewspace);
#endif

    glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_blur);
    glUniform2f(1,0,1.0f/float(height));
    glDrawArrays(GL_TRIANGLES,0,3);
  }


  void Sample::drawHbaoClassic(const Projection& projection, int width, int height, int sampleIdx)
  {
    prepareHbaoData(projection,width,height);

    drawLinearDepth(projection,width,height,sampleIdx);

    {
      NV_PROFILE_SECTION("ssaocalc");

      if (tweak.blur){
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
      }
      else{
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ZERO,GL_SRC_COLOR);
        if (tweak.samples > 1){
          glEnable(GL_SAMPLE_MASK);
          glSampleMaski(0, 1<<sampleIdx);
        }
      }

      glUseProgram(progManager.get( USE_AO_SPECIALBLUR && tweak.blur ? programs.hbao_calc_blur : programs.hbao_calc ));

      glBindBufferBase(GL_UNIFORM_BUFFER,0,buffers.hbao_ubo);
      glNamedBufferSubDataEXT(buffers.hbao_ubo,0,sizeof(HBAOData),&hbaoUbo);

      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);
      glBindMultiTextureEXT(GL_TEXTURE1, GL_TEXTURE_2D, textures.hbao_randomview[sampleIdx]);
      glDrawArrays(GL_TRIANGLES,0,3);
    }

    if (tweak.blur){
      drawHbaoBlur(projection,width,height,sampleIdx);
    }

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_SAMPLE_MASK);
    glSampleMaski(0, ~0);

    glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    glBindMultiTextureEXT(GL_TEXTURE1, GL_TEXTURE_2D, 0);

    glUseProgram(0);
  }

  void Sample::drawHbaoCacheAware(const Projection& projection, int width, int height, int sampleIdx)
  {
    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    prepareHbaoData(projection,width,height);

    drawLinearDepth(projection,width,height,sampleIdx);

    {
      NV_PROFILE_SECTION("viewnormal");
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.viewnormal);

      glUseProgram(progManager.get(programs.viewnormal));

      glUniform4fv(0, 1, hbaoUbo.projInfo.get_value());
      glUniform1i (1, hbaoUbo.projOrtho);
      glUniform2fv(2, 1, hbaoUbo.InvFullResolution.get_value());

      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);
      glDrawArrays(GL_TRIANGLES,0,3);
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    }

    {
      NV_PROFILE_SECTION("deinterleave");
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_deinterleave);
      glViewport(0,0,quarterWidth,quarterHeight);

      glUseProgram(progManager.get(programs.hbao2_deinterleave));
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);

      for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i+= NUM_MRT){
        glUniform4f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f, hbaoUbo.InvFullResolution.x, hbaoUbo.InvFullResolution.y);

        for (int layer = 0; layer < NUM_MRT; layer++){
          glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + layer, textures.hbao2_depthview[i+layer], 0);
        }
        glDrawArrays(GL_TRIANGLES,0,3);
      }
    }
    
    {
      NV_PROFILE_SECTION("ssaocalc");

      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_calc);
      glViewport(0,0,quarterWidth,quarterHeight);

      glUseProgram(progManager.get(USE_AO_SPECIALBLUR && tweak.blur ? programs.hbao2_calc_blur : programs.hbao2_calc));
      glBindMultiTextureEXT(GL_TEXTURE1, GL_TEXTURE_2D, textures.scene_viewnormal);

      glBindBufferBase(GL_UNIFORM_BUFFER,0,buffers.hbao_ubo);
      glNamedBufferSubDataEXT(buffers.hbao_ubo,0,sizeof(HBAOData),&hbaoUbo);

#if USE_AO_LAYERED_SINGLEPASS
      // instead of drawing to each layer individually
      // we draw all layers at once, and use image writes to update the array texture
      // this buys additional performance :)
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
      glBindImageTexture( 0, textures.hbao2_resultarray, 0, GL_TRUE, 0, GL_WRITE_ONLY, USE_AO_SPECIALBLUR ? GL_RG16F : GL_R8);
      glDrawArrays(GL_TRIANGLES,0,3 * HBAO_RANDOM_ELEMENTS);
      glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
#else
      for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
        glUniform2f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f);
        glUniform4fv(1, 1, hbaoRandom[i].get_value());

        glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao2_depthview[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textures.hbao2_resultarray, 0, i);

        glDrawArrays(GL_TRIANGLES,0,3);
      }
#endif
    }

    {
      NV_PROFILE_SECTION("reinterleave");

      if (tweak.blur){
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
      }
      else{
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ZERO,GL_SRC_COLOR);
        if (tweak.samples > 1){
          glEnable(GL_SAMPLE_MASK);
          glSampleMaski(0, 1<<sampleIdx);
        }
      }
      glViewport(0,0,width,height);

      glUseProgram(progManager.get(USE_AO_SPECIALBLUR && tweak.blur ? programs.hbao2_reinterleave_blur : programs.hbao2_reinterleave));

      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, textures.hbao2_resultarray);
      glDrawArrays(GL_TRIANGLES,0,3);
      glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, 0);
    }

    if (tweak.blur){
      drawHbaoBlur(projection,width,height,sampleIdx);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_SAMPLE_MASK);
    glSampleMaski(0,~0);

    glBindMultiTextureEXT(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    glBindMultiTextureEXT(GL_TEXTURE1, GL_TEXTURE_2D, 0);

    glUseProgram(0);


  }


  void Sample::think(double time)
  {
    m_control.processActions(m_window.m_viewsize,
      nv_math::vec2f(m_window.m_mouseCurrent[0],m_window.m_mouseCurrent[1]),
      m_window.m_mouseButtonFlags, m_window.m_wheel);

    if (m_window.onPress(KEY_R)){
      progManager.reloadPrograms();
    }
    if (!progManager.areProgramsValid()){
      waitEvents();
      return;
    }

    int width   = m_window.m_viewsize[0];
    int height  = m_window.m_viewsize[1];

    Projection projection;
    projection.update(width,height);

    if (tweakLast.samples != tweak.samples){
      initFramebuffers(width,height,tweak.samples);
    }
    tweakLast = tweak;

    {
      NV_PROFILE_SECTION("Scene");
      glViewport(0, 0, width, height);

      glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);

      nv_math::vec4   bgColor(0.2,0.2,0.2,0.0);
      glClearBufferfv(GL_COLOR,0,&bgColor.x);

      glClearDepth(1.0);
      glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);

      sceneUbo.viewport = uvec2(width,height);
     
      nv_math::mat4 view = m_control.m_viewMatrix;

      sceneUbo.viewProjMatrix = projection.matrix * view;
      sceneUbo.viewMatrix = view;
      sceneUbo.viewMatrixIT = nv_math::transpose(nv_math::invert(view));

      glUseProgram(progManager.get(programs.draw_scene));
      glBindBufferBase(GL_UNIFORM_BUFFER, UBO_SCENE, buffers.scene_ubo);
      glBufferSubData(GL_UNIFORM_BUFFER,0,sizeof(SceneData),&sceneUbo);

      glBindVertexBuffer(0,buffers.scene_vbo,0,sizeof(Vertex));
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers.scene_ibo);

      glEnableVertexAttribArray(VERTEX_POS);
      glEnableVertexAttribArray(VERTEX_NORMAL);
      glEnableVertexAttribArray(VERTEX_COLOR);

      glDrawElements(GL_TRIANGLES, sceneTriangleIndices, GL_UNSIGNED_INT, NV_BUFFER_OFFSET(0));

      glDisableVertexAttribArray(VERTEX_POS);
      glDisableVertexAttribArray(VERTEX_NORMAL);
      glDisableVertexAttribArray(VERTEX_COLOR);

      glBindBufferBase(GL_UNIFORM_BUFFER, UBO_SCENE, 0);
      glBindVertexBuffer(0,0,0,0);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    {
      NV_PROFILE_SECTION("ssao");

      for (int sample = 0; sample < tweak.samples; sample++)
      {
        switch(tweak.algorithm){
        case ALGORITHM_HBAO_CLASSIC:
          drawHbaoClassic(projection, width, height, sample);
          break;
        case ALGORITHM_HBAO_CACHEAWARE:
          drawHbaoCacheAware(projection, width, height, sample);
          break;
        }
      }
    }

    {
      NV_PROFILE_SECTION("Blit");
      // blit to background
      glBindFramebuffer(GL_READ_FRAMEBUFFER, fbos.scene);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      glBlitFramebuffer(0,0,width,height,
        0,0,width,height,GL_COLOR_BUFFER_BIT, GL_NEAREST);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    {
      NV_PROFILE_SECTION("TwDraw");
      TwDraw();
    }
  }

  void Sample::resize(int width, int height)
  {
    TwWindowSize(width,height);
    initFramebuffers(width,height,tweak.samples);
  }
}

using namespace ssao;

int sample_main(int argc, const char** argv)
{
  Sample sample;
  return sample.run(
    PROJECT_NAME,
    argc, argv,
    SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT,
    SAMPLE_MAJOR_VERSION, SAMPLE_MINOR_VERSION);
}
void sample_print(int level, const char * fmt)
{

}
