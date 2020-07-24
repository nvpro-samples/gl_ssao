/* Copyright (c) 2014-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Contact ckubisch@nvidia.com (Christoph Kubisch) for feedback */

#define DEBUG_FILTER     1

#include <include_gl.h>

#include <imgui/imgui_helper.h>
#include <imgui/imgui_impl_gl.h>

#include <nvmath/nvmath_glsltypes.h>

#include <nvh/geometry.hpp>
#include <nvh/misc.hpp>
#include <nvh/cameracontrol.hpp>

#include <nvgl/appwindowprofiler_gl.hpp>
#include <nvgl/error_gl.hpp>
#include <nvgl/programmanager_gl.hpp>
#include <nvgl/base_gl.hpp>

#include <noise/mersennetwister1.h>

// optimizes blur, by storing depth along with ssao calculation
// avoids accessing two different textures
#define USE_AO_SPECIALBLUR          1

// optimizes the cache-aware technique by rendering all temporary layers at once
// instead of individually

#define AO_LAYERED_OFF    0
#define AO_LAYERED_IMAGE  1
#define AO_LAYERED_GS     2

#define USE_AO_LAYERED_SINGLEPASS   AO_LAYERED_GS


#include "common.h"

namespace ssao
{
#if 0
  int const SAMPLE_SIZE_WIDTH(1280);
  int const SAMPLE_SIZE_HEIGHT(720);
#else
  int const SAMPLE_SIZE_WIDTH(1920);
  int const SAMPLE_SIZE_HEIGHT(1080);
#endif
  int const SAMPLE_MAJOR_VERSION(4);
  int const SAMPLE_MINOR_VERSION(5);

  static const int  NUM_MRT = 8;
  static const int  HBAO_RANDOM_SIZE = AO_RANDOMTEX_SIZE;
  static const int  HBAO_RANDOM_ELEMENTS = HBAO_RANDOM_SIZE*HBAO_RANDOM_SIZE;
  static const int  MAX_SAMPLES = 8;

  static const int        grid = 32;
  static const float      globalscale = 16.0f;

  class Sample : public nvgl::AppWindowProfilerGL
  {
    enum AlgorithmType {
      ALGORITHM_NONE,
      ALGORITHM_HBAO_CACHEAWARE,
      ALGORITHM_HBAO_CLASSIC,
      NUM_ALGORITHMS,
    };

    enum GuiEnums {
      GUI_ALGORITHM,
      GUI_MSAA,
    };

    struct {
      nvgl::ProgramID
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
      GLuint  scene = 0;
      GLuint  depthlinear = 0;
      GLuint  viewnormal = 0;
      GLuint  hbao_calc = 0;
      GLuint  hbao2_deinterleave = 0;
      GLuint  hbao2_calc = 0;
    } fbos;

    struct {
      GLuint  scene_vbo = 0;
      GLuint  scene_ibo = 0;
      GLuint  scene_ubo = 0;
      GLuint  hbao_ubo = 0;
    } buffers;

    struct {
      GLuint  scene_color = 0;
      GLuint  scene_depthstencil = 0;
      GLuint  scene_depthlinear = 0;
      GLuint  scene_viewnormal = 0;
      GLuint  hbao_result = 0;
      GLuint  hbao_blur = 0;
      GLuint  hbao_random = 0;
      GLuint  hbao_randomview[MAX_SAMPLES] = {0};
      GLuint  hbao2_deptharray = 0;
      GLuint  hbao2_depthview[HBAO_RANDOM_ELEMENTS] = {0};
      GLuint  hbao2_resultarray = 0;
    } textures;

    struct Vertex {

      Vertex(const nvh::geometry::Vertex& vertex) {
        position = vertex.position;
        normal = vertex.normal;
        color = nvmath::vec4(1.0f);
      }

      nvmath::vec4   position;
      nvmath::vec4   normal;
      nvmath::vec4   color;
    };


    struct Tweak {
      int             samples = 1;
      AlgorithmType   algorithm = ALGORITHM_HBAO_CACHEAWARE;
      float           intensity = 1.5f;
      float           bias = 0.1f;
      float           radius = 2.0f;
      float           blurSharpness = 40.0f;
      bool            blur = true;
      bool            ortho = false;
    };

    struct Projection {
      float nearplane = 0.1f;
      float farplane = 100.0f;
      float fov = 45.0f;
      float orthoheight = 1.0f;
      bool  ortho = false;
      mat4  matrix;

      void update(int width, int height) {
        float aspect = float(width) / float(height);
        if (ortho) {
          matrix = nvmath::ortho(-orthoheight*0.5f*aspect, orthoheight*0.5f*aspect, -orthoheight*0.5f, orthoheight*0.5f, nearplane, farplane);
        }
        else {
          matrix = nvmath::perspective(fov, aspect, nearplane, farplane);
        }
      }
    };

    ImGuiH::Registry  m_ui;
    double            m_uiTime = 0;

    nvgl::ProgramManager    m_progManager;
    nvh::CameraControl      m_control;

    Tweak             m_tweak;
    Tweak             m_tweakLast;

    uint              m_sceneTriangleIndices;
    uint              m_sceneObjects;

    Projection        m_projection;

    SceneData         m_sceneUbo;
    HBAOData          m_hbaoUbo;
    vec4f             m_hbaoRandom[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES];

    bool begin();
    void processUI(double time);
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

    void end() {
      ImGui::ShutdownGL();
    }
    // return true to prevent m_windowState updates
    bool mouse_pos(int x, int y) {
      return ImGuiH::mouse_pos(x, y);
    }
    bool mouse_button(int button, int action) {
      return ImGuiH::mouse_button(button, action);
    }
    bool mouse_wheel(int wheel) {
      return ImGuiH::mouse_wheel(wheel);
    }
    bool key_char(int button) {
      return ImGuiH::key_char(button);
    }
    bool key_button(int button, int action, int mods) {
      return ImGuiH::key_button(button, action, mods);
    }

  public:
    Sample() {
      m_parameterList.add("algorithm", (uint32_t*) &m_tweak.algorithm);
      m_parameterList.add("blur", &m_tweak.blur);
      m_parameterList.add("ortho", &m_tweak.ortho);
      m_parameterList.add("msaa", &m_tweak.samples);
    }
  };

  bool Sample::initProgram()
  {
    bool validated(true);
    m_progManager.m_filetype = nvh::ShaderFileManager::FILETYPE_GLSL;
    m_progManager.addDirectory( std::string("GLSL_" PROJECT_NAME));
    m_progManager.addDirectory( exePath() + std::string(PROJECT_RELDIRECTORY));

    m_progManager.registerInclude("common.h");

    m_progManager.m_prepend = nvgl::ProgramManager::format("#define AO_LAYERED %d\n", USE_AO_LAYERED_SINGLEPASS);

    programs.draw_scene = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "scene.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "scene.frag.glsl"));

    programs.bilateralblur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "bilateralblur.frag.glsl"));

    programs.depth_linearize = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define DEPTHLINEARIZE_MSAA 0\n", "depthlinearize.frag.glsl"));

    programs.depth_linearize_msaa = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define DEPTHLINEARIZE_MSAA 1\n", "depthlinearize.frag.glsl"));

    programs.viewnormal = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "viewnormal.frag.glsl"));

    programs.displaytex = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "displaytex.frag.glsl"));

    programs.hbao_calc = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 0\n", "hbao.frag.glsl"));

    programs.hbao_calc_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR_PRESENT 0\n","hbao_blur.frag.glsl"));

    programs.hbao_blur2 = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR_PRESENT 1\n","hbao_blur.frag.glsl"));

    programs.hbao2_calc = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
      nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,        "fullscreenquad.geo.glsl"),
#endif
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 0\n", "hbao.frag.glsl"));

    programs.hbao2_calc_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
      nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,        "fullscreenquad.geo.glsl"),
#endif
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao2_deinterleave = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "hbao_deinterleave.frag.glsl"));

    programs.hbao2_reinterleave = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR 0\n","hbao_reinterleave.frag.glsl"));

    programs.hbao2_reinterleave_blur = m_progManager.createProgram(
      nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,          "fullscreenquad.vert.glsl"),
      nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,        "#define AO_BLUR 1\n","hbao_reinterleave.frag.glsl"));

    validated = m_progManager.areProgramsValid();

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
      m_hbaoRandom[i].x = cosf(Angle);
      m_hbaoRandom[i].y = sinf(Angle);
      m_hbaoRandom[i].z = Rand2;
      m_hbaoRandom[i].w = 0;
#define SCALE ((1<<15))
      hbaoRandomShort[i*4+0] = (signed short)(SCALE*m_hbaoRandom[i].x);
      hbaoRandomShort[i*4+1] = (signed short)(SCALE*m_hbaoRandom[i].y);
      hbaoRandomShort[i*4+2] = (signed short)(SCALE*m_hbaoRandom[i].z);
      hbaoRandomShort[i*4+3] = (signed short)(SCALE*m_hbaoRandom[i].w);
#undef SCALE
    }

    nvgl::newTexture(textures.hbao_random, GL_TEXTURE_2D_ARRAY);
    glBindTexture(GL_TEXTURE_2D_ARRAY,textures.hbao_random);
    glTexStorage3D (GL_TEXTURE_2D_ARRAY,1,GL_RGBA16_SNORM,HBAO_RANDOM_SIZE,HBAO_RANDOM_SIZE,MAX_SAMPLES);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY,0,0,0,0, HBAO_RANDOM_SIZE,HBAO_RANDOM_SIZE,MAX_SAMPLES,GL_RGBA,GL_SHORT,hbaoRandomShort);
    glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D_ARRAY,0);

    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      glGenTextures(1, &textures.hbao_randomview[i]);
      glTextureView(textures.hbao_randomview[i], GL_TEXTURE_2D, textures.hbao_random, GL_RGBA16_SNORM, 0, 1, i, 1);
      glBindTexture(GL_TEXTURE_2D, textures.hbao_randomview[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glBindTexture(GL_TEXTURE_2D, 0);
    }

    nvgl::newBuffer(buffers.hbao_ubo);
    glNamedBufferStorage(buffers.hbao_ubo, sizeof(HBAOData), NULL, GL_DYNAMIC_STORAGE_BIT);

    return true;
  }

  bool Sample::initScene()
  {
    { // Scene Geometry
      nvh::geometry::Mesh<Vertex>  scene;
      const int LEVELS = 4;
      
      m_sceneObjects = 0;
      for (int i = 0; i < grid * grid; i++){

        vec4 color(nvh::frand(),nvh::frand(),nvh::frand(),1.0f);
        color *= 0.25f;
        color += 0.75f;

        vec2  posxy(i % grid, i / grid);
        
        float depth = sin(posxy.x*0.1f) * cos(posxy.y*0.1f) * 2.0f;


        for (int l = 0; l < LEVELS; l++){
          vec3  pos(posxy.x, posxy.y, depth);

          float scale = globalscale * 0.5f/float(grid);
          if (l != 0){
            scale *= powf(0.9f,float(l));
            scale *= nvh::frand()*0.5f + 0.5f;
          }

          vec3 size = vec3(scale);
          

          size.z *= nvh::frand()*1.0f+1.0f; 
          if (l != 0){
            size.z *= powf(0.7f,float(l));
          }

          pos -=  vec3( grid/2, grid/2, 0);
          pos /=  float(grid) / globalscale;

          depth += size.z;

          pos.z = depth;

          mat4  matrix    = nvmath::translation_mat4( pos) * nvmath::scale_mat4( size);

          uint  oldverts  = scene.getVerticesCount();
          uint  oldinds   = scene.getTriangleIndicesCount();

          nvh::geometry::Box<Vertex>::add(scene,matrix,2,2,2);

          for (uint v = oldverts; v < scene.getVerticesCount(); v++){
            scene.m_vertices[v].color = color;
          }

          depth += size.z;
        }

        m_sceneObjects++;
      }

      m_sceneTriangleIndices = scene.getTriangleIndicesCount();

      nvgl::newBuffer(buffers.scene_ibo);
      glNamedBufferStorage(buffers.scene_ibo, scene.getTriangleIndicesSize(), &scene.m_indicesTriangles[0], 0);

      nvgl::newBuffer(buffers.scene_vbo);
      glBindBuffer(GL_ARRAY_BUFFER, buffers.scene_vbo);
      glNamedBufferStorage(buffers.scene_vbo, scene.getVerticesSize(), &scene.m_vertices[0], 0);

      glVertexAttribFormat(VERTEX_COLOR,  4, GL_FLOAT, GL_FALSE,  offsetof(Vertex,color));
      glVertexAttribBinding(VERTEX_COLOR, 0);

      glVertexAttribFormat(VERTEX_POS,    3, GL_FLOAT, GL_FALSE,  offsetof(Vertex,position));
      glVertexAttribFormat(VERTEX_NORMAL, 3, GL_FLOAT, GL_FALSE,  offsetof(Vertex,normal));
      glVertexAttribBinding(VERTEX_POS,   0);
      glVertexAttribBinding(VERTEX_NORMAL,0);
    }

    { // Scene UBO
      nvgl::newBuffer(buffers.scene_ubo);
      glNamedBufferStorage(buffers.scene_ubo, sizeof(SceneData), NULL, GL_DYNAMIC_STORAGE_BIT);
    }

    return true;
  }

  bool Sample::initFramebuffers(int width, int height, int samples)
  {

    if (samples > 1){
      nvgl::newTexture(textures.scene_color, GL_TEXTURE_2D_MULTISAMPLE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, textures.scene_color);
      glTexStorage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_RGBA8, width, height, GL_FALSE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, 0);

      nvgl::newTexture(textures.scene_depthstencil, GL_TEXTURE_2D_MULTISAMPLE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, textures.scene_depthstencil);
      glTexStorage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_DEPTH24_STENCIL8, width, height, GL_FALSE);
      glBindTexture (GL_TEXTURE_2D_MULTISAMPLE, 0);
    }
    else
    {
      nvgl::newTexture(textures.scene_color, GL_TEXTURE_2D);
      glBindTexture (GL_TEXTURE_2D, textures.scene_color);
      glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
      glBindTexture (GL_TEXTURE_2D, 0);

      nvgl::newTexture(textures.scene_depthstencil, GL_TEXTURE_2D);
      glBindTexture (GL_TEXTURE_2D, textures.scene_depthstencil);
      glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, width, height);
      glBindTexture (GL_TEXTURE_2D, 0);
    }

    nvgl::newFramebuffer(fbos.scene);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.scene);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,        textures.scene_color, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, textures.scene_depthstencil, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    nvgl::newTexture(textures.scene_depthlinear, GL_TEXTURE_2D);
    glBindTexture (GL_TEXTURE_2D, textures.scene_depthlinear);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D, 0);

    nvgl::newFramebuffer(fbos.depthlinear);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.depthlinear);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,  textures.scene_depthlinear, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    nvgl::newTexture(textures.scene_viewnormal, GL_TEXTURE_2D);
    glBindTexture (GL_TEXTURE_2D, textures.scene_viewnormal);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D, 0);

    nvgl::newFramebuffer(fbos.viewnormal);
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
    
    nvgl::newTexture(textures.hbao_result, GL_TEXTURE_2D);
    glBindTexture (GL_TEXTURE_2D, textures.hbao_result);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture (GL_TEXTURE_2D, 0);

    nvgl::newTexture(textures.hbao_blur, GL_TEXTURE_2D);
    glBindTexture (GL_TEXTURE_2D, textures.hbao_blur);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture (GL_TEXTURE_2D, 0);

    nvgl::newFramebuffer(fbos.hbao_calc);
    glBindFramebuffer(GL_FRAMEBUFFER,     fbos.hbao_calc);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textures.hbao_result, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, textures.hbao_blur, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // interleaved hbao

    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    nvgl::newTexture(textures.hbao2_deptharray, GL_TEXTURE_2D_ARRAY);
    glBindTexture (GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32F, quarterWidth, quarterHeight, HBAO_RANDOM_ELEMENTS);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture (GL_TEXTURE_2D_ARRAY, 0);

    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
      if (textures.hbao2_depthview[i]) {
        glDeleteTextures(1, &textures.hbao2_depthview[i]);
      }
      glGenTextures(1, &textures.hbao2_depthview[i]);
      glTextureView(textures.hbao2_depthview[i], GL_TEXTURE_2D, textures.hbao2_deptharray, GL_R32F, 0, 1, i, 1);
      glBindTexture(GL_TEXTURE_2D, textures.hbao2_depthview[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glBindTexture(GL_TEXTURE_2D, 0);
    }


    nvgl::newTexture(textures.hbao2_resultarray, GL_TEXTURE_2D_ARRAY);
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

    nvgl::newFramebuffer(fbos.hbao2_deinterleave);
    glBindFramebuffer(GL_FRAMEBUFFER,fbos.hbao2_deinterleave);
    glDrawBuffers(NUM_MRT,drawbuffers);
    glBindFramebuffer(GL_FRAMEBUFFER,0);

    nvgl::newFramebuffer(fbos.hbao2_calc);
    glBindFramebuffer(GL_FRAMEBUFFER,fbos.hbao2_calc);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
    // this fbo will not have any attachments and therefore requires rasterizer to be configured
    // through default parameters
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH,  quarterWidth);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, quarterHeight);
#endif
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textures.hbao2_resultarray, 0);
#endif
    glBindFramebuffer(GL_FRAMEBUFFER,0);

    return true;
  }


  bool Sample::begin()
  {
    ImGuiH::Init(m_windowState.m_winSize[0], m_windowState.m_winSize[1], this);
    ImGui::InitGL();

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
    validated = validated && initFramebuffers(m_windowState.m_winSize[0],m_windowState.m_winSize[1],m_tweak.samples);

    m_ui.enumAdd(GUI_ALGORITHM, ALGORITHM_NONE, "none");
    m_ui.enumAdd(GUI_ALGORITHM, ALGORITHM_HBAO_CACHEAWARE, "hbao cache-aware");
    m_ui.enumAdd(GUI_ALGORITHM, ALGORITHM_HBAO_CLASSIC, "hbao classic");
    
    m_ui.enumAdd(GUI_MSAA, 1, "none");
    m_ui.enumAdd(GUI_MSAA, 2, "2x");
    m_ui.enumAdd(GUI_MSAA, 4, "4x");
    m_ui.enumAdd(GUI_MSAA, 8, "8x");
    
    m_control.m_sceneOrbit = vec3(0.0f);
    m_control.m_sceneDimension = float(globalscale);
    m_control.m_sceneOrthoZoom = m_control.m_sceneDimension;
    m_control.m_viewMatrix = nvmath::look_at(m_control.m_sceneOrbit - (vec3(0.4f,-0.35f,-0.6f)*m_control.m_sceneDimension*0.9f), m_control.m_sceneOrbit, vec3(0,1,0));
    
    m_projection.nearplane = m_control.m_sceneDimension * 0.01f;
    m_projection.farplane  = m_control.m_sceneDimension * 10.0f;


    return validated;
  }

  void Sample::processUI(double time)
  {
    int width = m_windowState.m_winSize[0];
    int height = m_windowState.m_winSize[1];

    // Update imgui configuration
    auto &imgui_io = ImGui::GetIO();
    imgui_io.DeltaTime = static_cast<float>(time - m_uiTime);
    imgui_io.DisplaySize = ImVec2(width, height);

    m_uiTime = time;

    ImGui::NewFrame();
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("NVIDIA " PROJECT_NAME, nullptr)) {
      m_ui.enumCombobox(GUI_MSAA, "msaa", &m_tweak.samples);
      m_ui.enumCombobox(GUI_ALGORITHM, "ssao algorithm", &m_tweak.algorithm);

      ImGui::Checkbox("orthographic", &m_tweak.ortho);
      ImGui::SliderFloat("radius", &m_tweak.radius, 0.0f, 4.0f);
      ImGui::SliderFloat("intensity", &m_tweak.intensity, 0.0f, 4.0f);
      ImGui::SliderFloat("bias", &m_tweak.bias, 0.0f, 0.9999f);
      ImGui::Checkbox("blur active", &m_tweak.blur);
      ImGui::SliderFloat("blur sharpness", &m_tweak.blurSharpness, 0.0f, 128.0f);
    }
    ImGui::End();
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
      2.0f / ( P[4*0+0]),      // ((x) * R - L)
      2.0f / ( P[4*1+1]),      // ((y) * T - B)
      -( 1.0f + P[4*3+0]) / P[4*0+0], // L
      -( 1.0f - P[4*3+1]) / P[4*1+1], // B
    };

    int useOrtho = projection.ortho ? 1 : 0;
    m_hbaoUbo.projOrtho = useOrtho;
    m_hbaoUbo.projInfo  = useOrtho ? projInfoOrtho : projInfoPerspective;

    float projScale;
    if (useOrtho){
      projScale = float(height) / (projInfoOrtho[1]);
    }
    else {
      projScale = float(height) / (tanf( projection.fov * 0.5f) * 2.0f);
    }

    // radius
    float meters2viewspace = 1.0f;
    float R = m_tweak.radius * meters2viewspace;
    m_hbaoUbo.R2 = R * R;
    m_hbaoUbo.NegInvR2 = -1.0f / m_hbaoUbo.R2;
    m_hbaoUbo.RadiusToScreen = R * 0.5f * projScale;

    // ao
    m_hbaoUbo.PowExponent = std::max(m_tweak.intensity,0.0f);
    m_hbaoUbo.NDotVBias = std::min(std::max(0.0f, m_tweak.bias),1.0f);
    m_hbaoUbo.AOMultiplier = 1.0f / (1.0f - m_hbaoUbo.NDotVBias);

    // resolution
    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    m_hbaoUbo.InvQuarterResolution = vec2(1.0f/float(quarterWidth),1.0f/float(quarterHeight));
    m_hbaoUbo.InvFullResolution = vec2(1.0f/float(width),1.0f/float(height));

#if USE_AO_LAYERED_SINGLEPASS
    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
      m_hbaoUbo.float2Offsets[i] = vec2(float(i % 4) + 0.5f, float(i / 4) + 0.5f);
      m_hbaoUbo.jitters[i] = m_hbaoRandom[i];
    }
#endif
  }

  void Sample::drawLinearDepth(const Projection& projection, int width, int height, int sampleIdx)
  {
    NV_PROFILE_GL_SECTION("linearize");
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);

    if (m_tweak.samples > 1){
      glUseProgram(m_progManager.get(programs.depth_linearize_msaa));
      glUniform4f(0, projection.nearplane * projection.farplane, projection.nearplane - projection.farplane, projection.farplane, projection.ortho ? 0.0f : 1.0f);
      glUniform1i(1,sampleIdx);

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, textures.scene_depthstencil);
      glDrawArrays(GL_TRIANGLES,0,3);
      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, 0);
    }
    else{
      glUseProgram(m_progManager.get(programs.depth_linearize));
      glUniform4f(0, projection.nearplane * projection.farplane, projection.nearplane - projection.farplane, projection.farplane, projection.ortho ? 0.0f : 1.0f);

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthstencil);
      glDrawArrays(GL_TRIANGLES,0,3);
      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    }
  }

  void Sample::drawHbaoBlur(const Projection& projection, int width, int height, int sampleIdx)
  {
    NV_PROFILE_GL_SECTION("ssaoblur");

    float meters2viewspace = 1.0f;

    glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR ? programs.hbao_blur : programs.bilateralblur));
    nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, textures.scene_depthlinear);

    glUniform1f(0,m_tweak.blurSharpness/meters2viewspace);

    glDrawBuffer(GL_COLOR_ATTACHMENT1);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_result);
    glUniform2f(1,1.0f/float(width),0);
    glDrawArrays(GL_TRIANGLES,0,3);

    // final output to main fbo
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ZERO,GL_SRC_COLOR);
    if (m_tweak.samples > 1){
      glEnable(GL_SAMPLE_MASK);
      glSampleMaski(0, 1<<sampleIdx);
    }

#if USE_AO_SPECIALBLUR
    glUseProgram(m_progManager.get(programs.hbao_blur2));
    glUniform1f(0,m_tweak.blurSharpness/meters2viewspace);
#endif

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao_blur);
    glUniform2f(1,0,1.0f/float(height));
    glDrawArrays(GL_TRIANGLES,0,3);
  }


  void Sample::drawHbaoClassic(const Projection& projection, int width, int height, int sampleIdx)
  {
    prepareHbaoData(projection,width,height);

    drawLinearDepth(projection,width,height,sampleIdx);

    {
      NV_PROFILE_GL_SECTION("ssaocalc");

      if (m_tweak.blur){
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
      }
      else{
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ZERO,GL_SRC_COLOR);
        if (m_tweak.samples > 1){
          glEnable(GL_SAMPLE_MASK);
          glSampleMaski(0, 1<<sampleIdx);
        }
      }

      glUseProgram(m_progManager.get( USE_AO_SPECIALBLUR && m_tweak.blur ? programs.hbao_calc_blur : programs.hbao_calc ));

      glBindBufferBase(GL_UNIFORM_BUFFER,0,buffers.hbao_ubo);
      glNamedBufferSubData(buffers.hbao_ubo,0,sizeof(HBAOData),&m_hbaoUbo);

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);
      nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, textures.hbao_randomview[sampleIdx]);
      glDrawArrays(GL_TRIANGLES,0,3);
    }

    if (m_tweak.blur){
      drawHbaoBlur(projection,width,height,sampleIdx);
    }

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_SAMPLE_MASK);
    glSampleMaski(0, ~0);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, 0);

    glUseProgram(0);
  }

  void Sample::drawHbaoCacheAware(const Projection& projection, int width, int height, int sampleIdx)
  {
    int quarterWidth  = ((width+3)/4);
    int quarterHeight = ((height+3)/4);

    prepareHbaoData(projection,width,height);

    drawLinearDepth(projection,width,height,sampleIdx);

    {
      NV_PROFILE_GL_SECTION("viewnormal");
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.viewnormal);

      glUseProgram(m_progManager.get(programs.viewnormal));

      glUniform4fv(0, 1, m_hbaoUbo.projInfo.get_value());
      glUniform1i (1, m_hbaoUbo.projOrtho);
      glUniform2fv(2, 1, m_hbaoUbo.InvFullResolution.get_value());

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);
      glDrawArrays(GL_TRIANGLES,0,3);
      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    }

    {
      NV_PROFILE_GL_SECTION("deinterleave");
      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_deinterleave);
      glViewport(0,0,quarterWidth,quarterHeight);

      glUseProgram(m_progManager.get(programs.hbao2_deinterleave));
      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.scene_depthlinear);

      for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i+= NUM_MRT){
        glUniform4f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f, m_hbaoUbo.InvFullResolution.x, m_hbaoUbo.InvFullResolution.y);

        for (int layer = 0; layer < NUM_MRT; layer++){
          glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + layer, textures.hbao2_depthview[i+layer], 0);
        }
        glDrawArrays(GL_TRIANGLES,0,3);
      }
    }
    
    {
      NV_PROFILE_GL_SECTION("ssaocalc");

      glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_calc);
      glViewport(0,0,quarterWidth,quarterHeight);

      glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR && m_tweak.blur ? programs.hbao2_calc_blur : programs.hbao2_calc));
      nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, textures.scene_viewnormal);

      glBindBufferBase(GL_UNIFORM_BUFFER,0,buffers.hbao_ubo);
      glNamedBufferSubData(buffers.hbao_ubo,0,sizeof(HBAOData),&m_hbaoUbo);

#if USE_AO_LAYERED_SINGLEPASS
      // instead of drawing to each layer individually
      // we draw all layers at once, and use image writes to update the array texture
      // this buys additional performance :)

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
      glBindImageTexture( 0, textures.hbao2_resultarray, 0, GL_TRUE, 0, GL_WRITE_ONLY, USE_AO_SPECIALBLUR ? GL_RG16F : GL_R8);
#endif
      glDrawArrays(GL_TRIANGLES,0,3 * HBAO_RANDOM_ELEMENTS);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
      glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
#endif
#else
      for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++){
        glUniform2f(0, float(i % 4) + 0.5f, float(i / 4) + 0.5f);
        glUniform4fv(1, 1, m_hbaoRandom[i].get_value());

        nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, textures.hbao2_depthview[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textures.hbao2_resultarray, 0, i);

        glDrawArrays(GL_TRIANGLES,0,3);
      }
#endif
    }

    {
      NV_PROFILE_GL_SECTION("reinterleave");

      if (m_tweak.blur){
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);
      }
      else{
        glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ZERO,GL_SRC_COLOR);
        if (m_tweak.samples > 1){
          glEnable(GL_SAMPLE_MASK);
          glSampleMaski(0, 1<<sampleIdx);
        }
      }
      glViewport(0,0,width,height);

      glUseProgram(m_progManager.get(USE_AO_SPECIALBLUR && m_tweak.blur ? programs.hbao2_reinterleave_blur : programs.hbao2_reinterleave));

      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, textures.hbao2_resultarray);
      glDrawArrays(GL_TRIANGLES,0,3);
      nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_ARRAY, 0);
    }

    if (m_tweak.blur){
      drawHbaoBlur(projection,width,height,sampleIdx);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_SAMPLE_MASK);
    glSampleMaski(0,~0);

    nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
    nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D, 0);

    glUseProgram(0);


  }


  void Sample::think(double time)
  {
    NV_PROFILE_GL_SECTION("Frame");

    m_control.m_sceneOrtho = m_tweak.ortho;
    m_control.processActions(m_windowState.m_winSize,
      nvmath::vec2f(m_windowState.m_mouseCurrent[0],m_windowState.m_mouseCurrent[1]),
      m_windowState.m_mouseButtonFlags, m_windowState.m_mouseWheel);

    if (m_windowState.onPress(KEY_R)){
      m_progManager.reloadPrograms();
    }
    if (!m_progManager.areProgramsValid()){
      waitEvents();
      return;
    }

    processUI(time);

    int width   = m_windowState.m_winSize[0];
    int height  = m_windowState.m_winSize[1];

    m_projection.ortho       = m_control.m_sceneOrtho;
    m_projection.orthoheight = m_control.m_sceneOrthoZoom;
    m_projection.update(width,height);

    if (m_tweakLast.samples != m_tweak.samples){
      initFramebuffers(width,height,m_tweak.samples);
    }
    m_tweakLast = m_tweak;

    {
      NV_PROFILE_GL_SECTION("Scene");
      glViewport(0, 0, width, height);

      glBindFramebuffer(GL_FRAMEBUFFER, fbos.scene);

      nvmath::vec4   bgColor(0.2,0.2,0.2,0.0);
      glClearBufferfv(GL_COLOR,0,&bgColor.x);

      glClearDepth(1.0);
      glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);

      m_sceneUbo.viewport = uvec2(width,height);
     
      nvmath::mat4 view = m_control.m_viewMatrix;

      m_sceneUbo.viewProjMatrix = m_projection.matrix * view;
      m_sceneUbo.viewMatrix = view;
      m_sceneUbo.viewMatrixIT = nvmath::transpose(nvmath::invert(view));

      glUseProgram(m_progManager.get(programs.draw_scene));
      glBindBufferBase(GL_UNIFORM_BUFFER, UBO_SCENE, buffers.scene_ubo);
      glBufferSubData(GL_UNIFORM_BUFFER,0,sizeof(SceneData),&m_sceneUbo);

      glBindVertexBuffer(0,buffers.scene_vbo,0,sizeof(Vertex));
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers.scene_ibo);

      glEnableVertexAttribArray(VERTEX_POS);
      glEnableVertexAttribArray(VERTEX_NORMAL);
      glEnableVertexAttribArray(VERTEX_COLOR);

      glDrawElements(GL_TRIANGLES, m_sceneTriangleIndices, GL_UNSIGNED_INT, NV_BUFFER_OFFSET(0));

      glDisableVertexAttribArray(VERTEX_POS);
      glDisableVertexAttribArray(VERTEX_NORMAL);
      glDisableVertexAttribArray(VERTEX_COLOR);

      glBindBufferBase(GL_UNIFORM_BUFFER, UBO_SCENE, 0);
      glBindVertexBuffer(0,0,0,0);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    {
      NV_PROFILE_GL_SECTION("ssao");

      for (int sample = 0; sample < std::max(1,m_tweak.samples); sample++)
      {
        switch(m_tweak.algorithm){
        case ALGORITHM_HBAO_CLASSIC:
          drawHbaoClassic(m_projection, width, height, sample);
          break;
        case ALGORITHM_HBAO_CACHEAWARE:
          drawHbaoCacheAware(m_projection, width, height, sample);
          break;
        }
      }
    }

    {
      NV_PROFILE_GL_SECTION("Blit");
      // blit to background
      glBindFramebuffer(GL_READ_FRAMEBUFFER, fbos.scene);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      glBlitFramebuffer(0,0,width,height,
        0,0,width,height,GL_COLOR_BUFFER_BIT, GL_NEAREST);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    {
      NV_PROFILE_GL_SECTION("GUI");
      ImGui::Render();
      ImGui::RenderDrawDataGL(ImGui::GetDrawData());
    }

    ImGui::EndFrame();
  }

  void Sample::resize(int width, int height)
  {
    initFramebuffers(width,height,m_tweak.samples);
  }
}

using namespace ssao;

int main(int argc, const char** argv)
{
  NVPSystem system(argv[0], PROJECT_NAME);
  Sample sample;
  return sample.run(
    PROJECT_NAME,
    argc, argv,
    SAMPLE_SIZE_WIDTH, SAMPLE_SIZE_HEIGHT);
}

