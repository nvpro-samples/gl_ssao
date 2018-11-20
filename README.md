# gl ssao

This sample implements screen space ambient occlusion (SSAO) using horizon-based ambient occlusion (HBAO). You can find some details about HBAO [here](http://www.nvidia.com/object/siggraph-2008-HBAO.html). It provides two alternative implementations the original hbao as well as an enhanced version that is more efficient in improved leveraging of the hardware's texture sampling cache, using [de-interleaved texturing](https://developer.nvidia.com/sites/default/files/akamai/gameworks/samples/DeinterleavedTexturing.pdf).

![sample screenshot](https://github.com/nvpro-samples/gl_ssao/blob/master/doc/sample.jpg)

> Note: This sample provides an improved HBAO algorithm, however it is not same as HBAO+ which is part of [NVIDIA ShadowWorks ](https://developer.nvidia.com/shadowworks) and improves the quality and performance of the algorithm further.


*HBAO - Classic*:
 - To achieve the effect a 4x4 texture that contains random directions is tiled across the screen and used to sample the neighborhood of a pixel's depth values.
 - The distance of the sampling depends on a customize-able world-size radius, for which the depth-buffer values are typically linearized first.
 - As the sampling radius depends on the pixel's depth, a big variability in the texture lookups can exist from one pixel to another. 
 - To reduce the costs the effect can be computed at lower-resolution and up-scaled for final display. As AO is typically a low-frequency effect this often can be sufficient.
 - Dithering artifacts can occur due to the 4x4 texture tiling. The image is blurred using cross-bilateral filtering that takes the depth values into account, to further improve quality.

*HBAO - Cache-Aware*:
 - The performance is vastly improved by grouping all pixels that share the same direction values. This means the screen-space linear depth buffer is stored in 16 layers each representing one direction of the 4x4 texture. Each layer has a quarter of the original resolution. The total amount of pixels is not reduced, but the sampling is performed in equal directions for the entire layer, yielding better texture cache utilization.
 - Linearizing the depth-buffer now stores into 16 texture layers.
 - The actual HBAO effect is performed in each layer individually, however all layers are independent of each other, allowing them to be processed in parallel.
 - Finally the results are stored scattered to their original locations in screen-space. 
 - Compared to the regular HBAO approach, the efficiency gains allow using the effect on full-resolution, improving the image quality. 

*MSAA support*:
 - The effect is run on a per-sample level N times (N matching the MSAA level). 
 - For each pass **glSampleMask( 1 << sample);** is used to update only the relevant samples in the target framebuffer.

*Blur*:
 - A cross-bilteral blur is used to eliminate the typical dithering artifacts. It makes use of the depth buffer to avoid smoothing over geometric discontinuities. 

![sample screenshot](https://github.com/nvpro-samples/gl_ssao/blob/master/doc/bluroff.jpg)

#### Performance

The cache-aware technique pays off on larger AO radii or higher resolutions (full HD).

Timings in microseconds via GL timer query taken on a Quadro M6000, no MSAA, 1080p (sample default is 720p, which may give less difference between the two).

*Classic*

```
Timer ssao;            GL    2434;
 Timer linearize;      GL      54;
 Timer ssaocalc;       GL    2177;
 Timer ssaoblur;       GL     198;
```

*Cache-Aware*

```
Timer ssao;            GL    1264;        
 Timer linearize;      GL      55;
 Timer viewnormal;     GL      76;
 Timer deinterleave;   GL      93;
 Timer ssaocalc;       GL     762;
 Timer reinterleave;   GL     100;
 Timer ssaoblur;       GL     167;
```

#### Sample Highlights

The user can change MSAA settings, blur settings and other parameters.

Key functionality is found in

- Sample::drawHbaoClassic()
- Sample::drawHbaoCacheAware()

As well as in helper functions

- Sample::drawLinearDepth()
- Sample::drawHbaoBlur()

The sample contains alternate codepaths for two additional optimizations, which are enabled by default.

* ```USE_AO_SPECIALBLUR```: Depth is stored with the ssao calculation, so that the blur can use a single instead of two texture fetches, which improves performance. 
* ```USE_AO_LAYERED_SINGLEPASS```: In the cache-aware technique we update the layers of the ssao calculation all at once using image stores and attachment-les fbo or a geometry shader with layers, instead of rendering to each layer individually.

### Building
Ideally clone this and other interesting [nvpro-samples](https://github.com/nvpro-samples) repositories into a common subdirectory. You will always need [shared_sources](https://github.com/nvpro-samples/shared_sources) and on Windows [shared_external](https://github.com/nvpro-samples/shared_external). The shared directories are searched either as subdirectory of the sample or one directory up. 

If you are interested in multiple samples, you can use [build_all](https://github.com/nvpro-samples/build_all) CMAKE as entry point, it will also give you options to enable/disable individual samples when creating the solutions.

### Providing Pull Requests
NVIDIA is happy to review and consider pull requests for merging into the main tree of the nvpro-samples for bug fixes and features. Before providing a pull request to NVIDIA, please note the following:

* A pull request provided to this repo by a developer constitutes permission from the developer for NVIDIA to merge the provided changes or any NVIDIA modified version of these changes to the repo. NVIDIA may remove or change the code at any time and in any way deemed appropriate.
* Not all pull requests can be or will be accepted. NVIDIA will close pull requests that it does not intend to merge.
The modified files and any new files must include the unmodified NVIDIA copyright header seen at the top of all shipping files.
