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

#version 430
/**/

#extension GL_ARB_shading_language_include : enable
#include "common.h"

layout(triangles) in;

#extension GL_NV_geometry_shader_passthrough : enable

#if GL_NV_geometry_shader_passthrough

  layout(passthrough) in gl_PerVertex {
    vec4 gl_Position;
  } gl_in[];
  layout(passthrough) in Inputs {
    vec2 texCoord;
  } IN[];
  
  void main()
  {
    gl_Layer = gl_PrimitiveIDIn;
    gl_PrimitiveID = gl_PrimitiveIDIn;
  }

#else
  
  layout(triangle_strip,max_vertices=3) out;

  in Inputs {
    vec2 texCoord;
  } IN[];
  out vec2 texCoord;

  void main()
  {
    for (int i = 0; i < 3; i++){
      texCoord = IN[i].texCoord;
      gl_Layer = gl_PrimitiveIDIn;
      gl_PrimitiveID = gl_PrimitiveIDIn;
      gl_Position = gl_in[i].gl_Position;
      EmitVertex();
    }
  }

#endif
