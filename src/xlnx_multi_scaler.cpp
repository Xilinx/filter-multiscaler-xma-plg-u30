/*       
 * Copyright (C) 2019, Xilinx Inc - All rights reserved
 * Xilinx Multiscaler XMA Plugin 
 *                                    
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations 
 * under the License.
 */        
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <inttypes.h>
#include <xma.h>
#include <xmaplugin.h>
#include <syslog.h>
#include "xv_multi_scaler_hw.h"
#include "xlnx_abr_scaler_coeffs.h"
#include "xlnx_fixed_scaler_coeffs.h"

#undef MEASURE_TIME
#ifdef MEASURE_TIME
#define MAX_COUNT_TIME 1000
#include <time.h>
#endif

#include <xvbm.h>
/* #define XVBM_BUFF_PR(...) printf(__VA_ARGS__) */
#define XVBM_BUFF_PR(...)

//#define DEBUG_MULTISCALE

#define MAX_OUTPOOL_BUFFERS   7
#define MAX_PIPELINE_BUFFERS  2

#undef DUMP_INPUT_FRAMES

#define MAX_VPLANES       2 // MAX planes supported by multiscaler is 2 by v2019.1
#define MAX_FRAMERATE     60
#define MAX_OUTPUTS       8
#define MULTISCALER_PPC   4
/* #define MULTISCALER_WIDTH_BYTES 16 // for 2ppc */
/* #define MULTISCALER_WIDTH_BYTES 32 // for 4ppc */
#define VCU_WIDTH_ALIGN   256
#define VCU_HEIGHT_ALIGN  64

#define MAX_WIDTH         3840
#define MAX_HEIGHT        2160
#define MAX_PIXELS        (MAX_WIDTH * MAX_HEIGHT)

#define MULTISCALER_ALIGN(stride,MMWidthBytes)  ((((stride)+(MMWidthBytes)-1)/(MMWidthBytes))*(MMWidthBytes))
#define ALIGN(width,align)                      (((width) + (align) - 1) & ~((align) - 1))

#ifdef DUMP_INPUT_FRAMES
FILE *infp = NULL;
#endif

#define XMA_MULTISCALER "xma-multiscaler"

#define ERROR_PRINT(...) {\
  do {\
    fprintf(stderr, "[%s:%d] ERROR : ",__func__, __LINE__);\
    fprintf(stderr, __VA_ARGS__);\
    fprintf(stderr, "\n");\
    char msg1[1024];\
    sprintf(msg1, __VA_ARGS__);\
    char msg[1024];\
    sprintf(msg, "[%s:%d] ERROR : %s", __func__, __LINE__, msg1);\
    xma_logmsg(XMA_ERROR_LOG, XMA_MULTISCALER, msg);\
  } while(0);\
}

#define DEBUG_PRINT(...) {\
  do {\
    char msg1[1024];\
    sprintf(msg1, __VA_ARGS__);\
    char msg[1024];\
    sprintf(msg, "[%s:%d] %s", __func__, __LINE__, msg1);\
    xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, msg);\
  } while(0);\
}

typedef struct {
  uint32_t widthIn;
  uint32_t widthOut;
  uint32_t heightIn;
  uint32_t heightOut;
  uint32_t lineRate;
  uint32_t pixelRate;
  uint32_t inPixelFmt;
  uint32_t outPixelFmt;
  uint32_t strideIn;
  uint32_t strideOut;
  uint64_t srcImgBuf[3];
  uint64_t dstImgBuf[3];
  uint64_t hfltCoeffAddr;
  uint64_t vfltCoeffAddr;
  uint64_t nxtaddr;
} XV_MULTISCALER_DESCRIPTOR;

typedef enum
{
  XV_MULTI_SCALER_NONE        = -1,
  XV_MULTI_SCALER_RGBX8       = 10,
  XV_MULTI_SCALER_YUVX8       = 11,
  XV_MULTI_SCALER_YUYV8       = 12,
  XV_MULTI_SCALER_RGBX10      = 15,
  XV_MULTI_SCALER_YUVX10      = 16,
  XV_MULTI_SCALER_Y_UV8       = 18,
  XV_MULTI_SCALER_Y_UV8_420   = 19, /* NV12 */
  XV_MULTI_SCALER_RGB8        = 20,
  XV_MULTI_SCALER_YUV8        = 21,
  XV_MULTI_SCALER_Y_UV10      = 22,
  XV_MULTI_SCALER_Y_UV10_420  = 23,
  XV_MULTI_SCALER_Y8          = 24,
  XV_MULTI_SCALER_Y10         = 25,
  XV_MULTI_SCALER_BGRX8       = 27,
  XV_MULTI_SCALER_UYVY8       = 28,
  XV_MULTI_SCALER_BGR8        = 29, /* BGR */
} XV_MULTISCALER_MEMORY_FORMATS;

typedef enum {
    XV_MULTI_SCALER_XPID_EN_PIPELINE = 0,
    XV_MULTI_SCALER_XPID_LOGLVL,
    XV_MULTI_SCALER_XPID_MIXRATE_SESSION,
    XV_MULTI_SCALER_XPID_NUM_PARAMS
}XV_MULTISCALER_XPARAM_INDEX;

typedef enum {
  XLXN_FIXED_COEFF_SR1,
  XLXN_FIXED_COEFF_SR105,
  XLXN_FIXED_COEFF_SR115,
  XLXN_FIXED_COEFF_SR125,
  XLXN_FIXED_COEFF_SR14,
  XLXN_FIXED_COEFF_SR15,
  XLXN_FIXED_COEFF_SR175,
  XLXN_FIXED_COEFF_SR19,
  XLXN_FIXED_COEFF_SR2,
  XLXN_FIXED_COEFF_SR225,
  XLXN_FIXED_COEFF_SR25,
  XLXN_FIXED_COEFF_SR3,   
  XLXN_FIXED_COEFF_TAPS_6,
  XLXN_FIXED_COEFF_TAPS_12,  
} XLNX_FIXED_FILTER_COEFF_TYPE;

enum
{
  XMA_COEFF_AUTO_GENERATE,
  XMA_COEFF_USE_DEFAULT,
  XMA_COEFF_LOAD_FROM_FILE,
};

typedef struct MultiScalerContext
{
  uint32_t            enable_pipeline;
  uint8_t             num_outs;
  uint16_t            in_height[MAX_OUTPUTS];
  uint16_t            in_width[MAX_OUTPUTS];
  uint16_t            out_height[MAX_OUTPUTS];
  uint16_t            out_width[MAX_OUTPUTS];
  XV_MULTISCALER_MEMORY_FORMATS in_format[MAX_OUTPUTS];
  XV_MULTISCALER_MEMORY_FORMATS out_format[MAX_OUTPUTS];
  uint32_t            pixel_rate[MAX_OUTPUTS];
  uint32_t            line_rate[MAX_OUTPUTS];
  uint32_t            in_stride[MAX_OUTPUTS];
  uint32_t            out_stride[MAX_OUTPUTS];
  ScalerFilterCoeffs  FilterCoeffs[MAX_OUTPUTS];
  XvbmPoolHandle      in_phandle;
  XvbmPoolHandle      out_phandle[MAX_OUTPUTS][MAX_VPLANES];
  XvbmBufferHandle    out_bhandle[MAX_OUTPUTS][MAX_OUTPOOL_BUFFERS][MAX_VPLANES];
  XvbmBufferHandle    in_bhandle[MAX_OUTPOOL_BUFFERS];
  XmaBufferObj        HfltCoeff_Buffer[MAX_OUTPUTS];
  XmaBufferObj        VfltCoeff_Buffer[MAX_OUTPUTS];
  XmaFraction         time_base[MAX_OUTPOOL_BUFFERS];
  XmaFraction         frame_rate[MAX_OUTPOOL_BUFFERS];
  uint64_t            pts[MAX_OUTPOOL_BUFFERS];
  int32_t             is_idr[MAX_OUTPOOL_BUFFERS];
  int                 s_idx;
  int                 r_idx;
  uint64_t            recv_frame_cnt;
  uint64_t            sent_frame_cnt;
  bool                pool_extended;
  int8_t              current_pipe;
  int8_t              first_frame;
  XmaScalerSession    *session_mix_rate;
#ifdef MEASURE_TIME
  int send_count;
  int recv_count;
  long long int send_func_time;
  long long int recv_func_time;
  long long int send_xrt_time;
  long long int recv_xrt_time;
#endif
  long long int     frame_sent;
  long long int     frame_recv;
  struct timespec   latency;
  long long int     time_taken;
  int               latency_logging;
  uint8_t                   hw_reg[XV_MULTI_SCALER_CTRL_REGMAP_SIZE];
  XV_MULTISCALER_DESCRIPTOR *desc;
  XmaBufferObj              desc_buffer[MAX_OUTPUTS];
} MultiScalerContext;

static XmaParameter* get_parameter (XmaParameter *params, int num_params, const char  *name)
{
  int i = 0;
  for (i = 0; i < num_params; ++i)
  {
    if (!strcmp(name, params[i].name))
      return &params[i];
  }
  return NULL;
}

static void get_user_params(XmaScalerSession *session)
{
   XmaParameter *param;
   MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;

  // With zero copy the pipeline option is not currently used in U30 .
  // Disabled scaler pipeline option for now, as the existing pipeline causes issues in reading buffer. 
  // It is sporadically reading old buffer state after xma_plg_is_work_item_done than the new and the issue is not limited to first few frames. 
  //if ((param = get_parameter (session->props.params, session->props.param_cnt, "enable_pipeline")))
  //     ctx->enable_pipeline = *(uint32_t*)param->value;
  ctx->enable_pipeline = 0;

  if ((param = get_parameter (session->props.params, session->props.param_cnt, "MixRate")))
       ctx->session_mix_rate = (XmaScalerSession *)*(uint64_t *)param->value;

  if ((param = get_parameter (session->props.params, session->props.param_cnt, "latency_logging")))
       ctx->latency_logging = (int)*(int *)param->value;
  else
      ctx->latency_logging = 0;
}


/* Converts XMA color fmt to Multi-Scaler IP format */
static XV_MULTISCALER_MEMORY_FORMATS
get_multiscaler_ip_format (XmaFormatType app_format)
{
  switch (app_format) {
//    case XMA_RGB888_FMT_TYPE:
//      return XV_MULTI_SCALER_BGR8;
//    case XMA_YUV420_FMT_TYPE:
    case XMA_VCU_NV12_FMT_TYPE:
      return XV_MULTI_SCALER_Y_UV8_420;
    default:
      ERROR_PRINT ("Unsupported format...");
      return XV_MULTI_SCALER_NONE;
  }
}

static int32_t
get_num_video_planes (XmaFormatType format)
{
  /* multi scaler supports max 2 planes till v2019.1 */
  switch (format) {
//    case XMA_RGB888_FMT_TYPE: /* BGR */
//      return 1;
//    case XMA_YUV420_FMT_TYPE: /* YUV420P */
//      return 3;
    case XMA_VCU_NV12_FMT_TYPE: /* NV12 or VCU_NV12 */
      return 1;
    default:
      ERROR_PRINT ("Unsupported format...");
      return -1;
  }
}

static int32_t
get_plane_size (int32_t stride, int32_t height, XmaFormatType format, int32_t plane_id)
{
  switch (format) {
//    case XMA_RGB888_FMT_TYPE: /* BGR */
//      if (plane_id > 0) {
//        ERROR_PRINT ("Wrong plane ID for RGB888...");
//        return -1;
//      }
//      return (stride * height);
//    case XMA_YUV420_FMT_TYPE: /* YUV420P */
//      switch (plane_id) {
//        case 0: /*plane 0*/
//          return stride * ALIGN(height, VCU_HEIGHT_ALIGN);
//        case 1: /*plane 1*/
//        case 2: /*plane 2*/
//          return ((stride * ALIGN(height, VCU_HEIGHT_ALIGN)) >> 2);
//        default:
//          ERROR_PRINT ("Wrong plane ID for NV12...");
//          return -1;
//      }
    case XMA_VCU_NV12_FMT_TYPE: /* VCU_NV12 */
      if (plane_id > 0) {
        ERROR_PRINT ("Wrong plane ID for VCU_NV12...");
        return -1;
      }
      //return ((stride * ALIGN(height, VCU_HEIGHT_ALIGN)) + ((stride * height) >> 1));
      return ((stride * ALIGN(height, VCU_HEIGHT_ALIGN) * 3)>>1);
    default:
      ERROR_PRINT ("Unsupported format...");
      return -1;
  }
}

static void copy_filt_set(int16_t dest_filt[64][12], int set)
{
    int i=0, j=0;

    for ( i=0; i<64; i++)
    {
        for ( j=0; j<12; j++)
        {
           switch(set)
           {   
              case XLXN_FIXED_COEFF_SR1:
                      dest_filt[i][j] = fixed_coeff_SR1[i][j];//SR1.0
                      break;
              case XLXN_FIXED_COEFF_SR105: 
                      dest_filt[i][j] = fixed_coeff_SR1_05[i][j]; //SR1.05
                      break;
              case XLXN_FIXED_COEFF_SR115: 
                      dest_filt[i][j] = fixed_coeff_SR1_15[i][j]; //SR1.15 
                      break;
              case XLXN_FIXED_COEFF_SR125: 
                      dest_filt[i][j] = fixed_coeff_SR1_25[i][j]; //SR2.5
                      break;
              case XLXN_FIXED_COEFF_SR14: 
                      dest_filt[i][j] = fixed_coeff_SR1_4[i][j]; //SR1.4
                      break;
              case XLXN_FIXED_COEFF_SR175:
                      dest_filt[i][j] = fixed_coeff_SR1_75[i][j]; //SR1.75
                      break;
              case XLXN_FIXED_COEFF_SR19:
                      dest_filt[i][j] = fixed_coeff_SR1_9[i][j]; //SR1.9
                      break;
              case XLXN_FIXED_COEFF_SR2:
                      dest_filt[i][j] = fixed_coeff_SR2_0[i][j]; //SR2.0
                      break;	
              case XLXN_FIXED_COEFF_SR225:
                      dest_filt[i][j] = fixed_coeff_SR2_25[i][j]; //SR2.25
                      break;	
              case XLXN_FIXED_COEFF_SR25:
                      dest_filt[i][j] = fixed_coeff_SR2_5[i][j]; //SR2.5
                      break;					  
              case XLXN_FIXED_COEFF_SR3:
                      dest_filt[i][j] = fixed_coeff_SR3_0[i][j]; //SR3.0
                      break;					  
              case XLXN_FIXED_COEFF_TAPS_6:
                      dest_filt[i][j] = fixed_coeff_taps6in12[i][j]; //6tap: Always used for up scale
                      break;
               case XLXN_FIXED_COEFF_TAPS_12:
                      dest_filt[i][j] = fixed_coeff_taps12[i][j]; //smooth filter
                      break;             					  
              default:		
                      dest_filt[i][j] = fixed_coeff_taps12[i][j]; //12tap
                      break;		
		   }
        }		   
    }
}

static int Select_filt_id(float down_scale_ratio)
{
    int filter_id = XLXN_FIXED_COEFF_TAPS_12;

    if (down_scale_ratio == 1)
       filter_id = XLXN_FIXED_COEFF_SR1;
    else if (down_scale_ratio < 1.10) 
       filter_id = XLXN_FIXED_COEFF_SR105;
    else if (down_scale_ratio < 1.20) 
       filter_id = XLXN_FIXED_COEFF_SR115;
    else if (down_scale_ratio < 1.30) 
       filter_id = XLXN_FIXED_COEFF_SR125;
    else if (down_scale_ratio < 1.45) 
       filter_id = XLXN_FIXED_COEFF_SR14;
    else if (down_scale_ratio < 1.60) 
       filter_id = XLXN_FIXED_COEFF_SR15;
    else if (down_scale_ratio < 1.80) 
       filter_id = XLXN_FIXED_COEFF_SR175;	
    else if (down_scale_ratio < 1.95) 
       filter_id = XLXN_FIXED_COEFF_SR19;
    else if (down_scale_ratio < 2.15) 
       filter_id = XLXN_FIXED_COEFF_SR2;
    else if (down_scale_ratio < 2.24) 
       filter_id = XLXN_FIXED_COEFF_SR225;
    else if (down_scale_ratio < 2.6) 
       filter_id = XLXN_FIXED_COEFF_SR25;	
    else if (down_scale_ratio < 3.5) 
       filter_id = XLXN_FIXED_COEFF_SR3;	  
	  
    return filter_id;
	
}


static int32_t
xlnx_multi_scaler_prepare_filter_tables (XmaScalerSession *session)
{
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  float scale_ratio[2][MAX_OUTPUTS] = {{0.0}};
  int upscale_enable[2][MAX_OUTPUTS] = {0};
  int filterSet[2][MAX_OUTPUTS]={{0}};
  int rt[2][MAX_OUTPUTS] = {{-1}};
  int output_id = 0;
  int64_t B = 0 * (1 << 24);
  int64_t C = 0.6 * (1 << 24);
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);
  int filterSize=0;
  int load_coeff_file = false;
  int temp = 0, r=1, d, i, j;
  FILE *fp_coeff = NULL;

  for (output_id = 0; output_id < max_outputs; output_id++) {
    /* store width scaling ratio  */
    if (ctx->in_width[output_id] >= ctx->out_width[output_id]) {
      scale_ratio[0][output_id] = ((float)ctx->in_width[output_id]/(float)ctx->out_width[output_id]); //downscale
    } else {
      scale_ratio[0][output_id] = ((float)ctx->out_width[output_id]/(float)ctx->in_width[output_id]); //upscale
      upscale_enable[0][output_id] = 1;
    }

    /* store height scaling ratio */
    if (ctx->in_height[output_id] >= ctx->out_height[output_id]) {
      scale_ratio[1][output_id] = ((float)ctx->in_height[output_id]/(float)ctx->out_height[output_id]); //downscale
    } else {
      scale_ratio[1][output_id] = ((float)ctx->out_height[output_id]/(float)ctx->in_height[output_id]);//upscale
      upscale_enable[1][output_id] = 1;
    }

    for (d=0; d<2; d++) {
      if (upscale_enable[d][output_id]==1) {
        /* upscaling default use 6 taps */
        filterSet[d][output_id] = XLXN_FIXED_COEFF_TAPS_6;
      } else {
        /*Get index of downscale fixed filter*/
        filterSet[d][output_id] = Select_filt_id(scale_ratio[d][output_id]);
      }
      DEBUG_PRINT ("channel = %d, %s scaling ratio = %f and chosen filter type = %d", output_id, d == 0 ? "width" : "height",
          scale_ratio[d][output_id], filterSet[d][output_id]);
    }

    // TODO: loading coefficients default is not handled.. need to check why not in abr
    if (session->props.output[output_id].coeffLoad == XMA_COEFF_AUTO_GENERATE) {
      /* Auto generate cardinal cubic coefficients when coeffLoad is 0 */
      rt[0][output_id] = feasibilityCheck(ctx->in_width[output_id], ctx->out_width[output_id],
          &filterSize);
      if ((rt[0][output_id]==0) && (upscale_enable[0][output_id]!=1)) {
        DEBUG_PRINT ("Generate cardinal cubic horizontal coefficients");
        Generate_cardinal_cubic_spline(ctx->in_width[output_id], ctx->out_width[output_id],
            filterSize, B, C, (int16_t *)ctx->FilterCoeffs[output_id].HfltCoeff);
      } else {
        /* get fixed horizontal filters*/
        DEBUG_PRINT ("Consider predefined horizontal filter coefficients");
        copy_filt_set((ctx->FilterCoeffs[output_id].HfltCoeff), filterSet[0][output_id]);
      }
      /* horizontal filters */
      rt[1][output_id] = feasibilityCheck(ctx->in_height[output_id], ctx->out_height[output_id],
          &filterSize);
      if ((rt[1][output_id]==0) &&  (upscale_enable[1][output_id]!=1)) {
        DEBUG_PRINT ("Generate cardinal cubic vertical coefficients");
        Generate_cardinal_cubic_spline(ctx->in_height[output_id],
            ctx->out_height[output_id], filterSize, B, C,
            (int16_t *)ctx->FilterCoeffs[output_id].VfltCoeff);
      } else {
        /* get fixed vertical filters*/
        DEBUG_PRINT ("Consider predefined vertical filter coefficients");
        copy_filt_set((ctx->FilterCoeffs[output_id].VfltCoeff), filterSet[1][output_id]);
      }
    } else if ((session->props.output[output_id].coeffLoad == XMA_COEFF_LOAD_FROM_FILE)) {
      load_coeff_file = true;
    } else { //XMA_COEFF_USE_DEFAULT
      /* get fixed horizontal filters*/
      DEBUG_PRINT ("Consider predefined horizontal filter coefficients");

      copy_filt_set((ctx->FilterCoeffs[output_id].HfltCoeff), filterSet[0][output_id]);
      /* get fixed vertical filters*/
      DEBUG_PRINT ("Consider predefined vertical filter coefficients");
      copy_filt_set((ctx->FilterCoeffs[output_id].VfltCoeff), filterSet[1][output_id]);
    }
  }

  if (load_coeff_file) {
    xma_logmsg(XMA_INFO_LOG, XMA_MULTISCALER, " --------------------------------------------------------------");
    xma_logmsg(XMA_INFO_LOG, XMA_MULTISCALER, "INFO: Expects to load from 'FilterCoeff.txt'."
        "For all output resolutions coeffLoad is set to 2,  "
        "the coefficient file must have \n\t all horizontal coefficient sets "
        "followed by all vertical sets.\n\tEach set is expected to have "
        "64 phases of 12 taps seperated by space.");
    xma_logmsg(XMA_INFO_LOG, XMA_MULTISCALER, " --------------------------------------------------------------");

    fp_coeff = fopen(session->props.input.coeffFile, "rb");
    if (fp_coeff == NULL) {
      ERROR_PRINT ("failed to open file %s for reading. reason : %s [%d]",
          session->props.input.coeffFile, strerror(errno), errno);
      return XMA_ERROR;
    }
  }

  for (output_id=0; output_id < max_outputs; output_id++) {
    for (i=0; i < HSC_PHASES; i++) {
      for (j=0; j < HSC_TAPS; j++) {
        /* load horizontal filters from file when specific out index requires */
        if ((session->props.output[output_id].coeffLoad == XMA_COEFF_LOAD_FROM_FILE) && load_coeff_file) {
          r &= (fscanf(fp_coeff, "%d", &temp)!=EOF);
          ctx->FilterCoeffs[output_id].HfltCoeff[i][j] = (int16_t) temp;
        }
      }
    }
  }

  for (output_id=0; output_id < max_outputs; output_id++) {
    for (i=0; i<VSC_PHASES; i++) {
      for (j=0; j<VSC_TAPS; j++) {
        /* load horizontal filters from file when specific out index requires */
        if ((session->props.output[output_id].coeffLoad == XMA_COEFF_LOAD_FROM_FILE) && load_coeff_file) {
          r &= (fscanf(fp_coeff, "%d", &temp)!=EOF);
          ctx->FilterCoeffs[output_id].VfltCoeff[i][j] = (int16_t) temp;
        }
      }
    }
  }

  if (r <= 0) {
    ERROR_PRINT ("Coefficient file %s doesn't have enough coefficients. Exiting the application.",session->props.input.coeffFile);
    fclose(fp_coeff);
    return XMA_ERROR;
  }

  if(load_coeff_file)
    fclose(fp_coeff);

  return XMA_SUCCESS;
}

static int32_t
multi_scaler_allocate_buffers (XmaScalerSession *session)
{
  XmaSession xma_session = session->base;
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  int output_id = 0, ret=0;
  int plane_id;
  int max_outputs   = MIN(ctx->num_outs, MAX_OUTPUTS);
  int ddr_bank_index = xma_session.hw_session.bank_index;  
  size_t        b_size;
  XmaBufferObj  bo_handle;
  XvbmPoolHandle  p_handle;

  /* allocate input buffers */
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Allocate buffer pool for input ->");

  b_size = ((ALIGN(session->props.input.width,  VCU_WIDTH_ALIGN) * 
             ALIGN(session->props.input.height, VCU_HEIGHT_ALIGN)) * 3) >> 1;

  p_handle = xvbm_buffer_pool_create(xma_plg_get_dev_handle(xma_session),
                                     1,
                                     b_size,
                                     ddr_bank_index);
  if (!p_handle) {
    ERROR_PRINT("Input buffer pool create failed\n");
    return XMA_ERROR;
  }
  ctx->in_phandle = p_handle;
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "    Created Pool %p (%4d x %4d)\n",ctx->in_phandle,
                  ALIGN(session->props.input.width,  VCU_WIDTH_ALIGN),
                  ALIGN(session->props.input.height, VCU_HEIGHT_ALIGN));

  /* allocate output buffers */
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Allocate buffer pool for output channels ->");

  for (output_id = 0; output_id < max_outputs; output_id++) {
    xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "    [Chan id=%d]", output_id);

    for (plane_id = 0; plane_id < get_num_video_planes(session->props.output[output_id].format); plane_id++) {
      b_size = get_plane_size(ctx->out_stride[output_id],
                              ctx->out_height[output_id],
                              session->props.output[output_id].format,
                              plane_id);
      /* For normal session allocate buffers from new pool */
      if (!ctx->session_mix_rate) {
          p_handle = xvbm_buffer_pool_create(xma_plg_get_dev_handle(xma_session),
                                             MAX_OUTPOOL_BUFFERS,
                                             b_size,
                                             ddr_bank_index);
          if (!p_handle) {
             ERROR_PRINT ("Output buffer pool create failed\n");
             goto cleanup;
          }
          ctx->out_phandle[output_id][plane_id] = p_handle;
          xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "      [plane_id: %d]:: Created Pool %p (%4d x %4d)\n",plane_id,
                          ctx->out_phandle[output_id][plane_id],
                          ctx->out_stride[output_id],
                          ALIGN(ctx->out_height[output_id], VCU_HEIGHT_ALIGN));
      } else {
        /* For mix_rate session extend the pool allocated in previous session */
        MultiScalerContext *mixrate_ctx = (MultiScalerContext *)ctx->session_mix_rate->base.plugin_data;
        XvbmPoolHandle shared_pool = NULL;
        XvbmBufferHandle buffer    = NULL;
        size_t prev_size           = 0;
        int num = 0;

        shared_pool = mixrate_ctx->out_phandle[output_id][plane_id];
        if (shared_pool) {
            //pool for requested output_id exists. Join sessions by extending existing pool
            xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Chan_id = %d :: Shared Pool %p Exists with %d buffers\n",
                      output_id,shared_pool,xvbm_get_freelist_count(shared_pool));
            buffer    = xvbm_get_buffer_handle(shared_pool, 0);
            prev_size = xvbm_buffer_get_size(buffer);
            //ensure buffers in prev_pool is of same size as requested for this session
            if (prev_size == b_size) {
                int cnt = xvbm_buffer_pool_num_buffers_get(buffer);
                num     = xvbm_buffer_pool_extend(buffer, MAX_OUTPOOL_BUFFERS);
                if (num == (cnt + MAX_OUTPOOL_BUFFERS)) {
                    xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Chan_id = %d :: Extended Shared pool to %d buffers\n",output_id,num);
                } else {
                    ERROR_PRINT("%s : scaler pool extension failed\n", __func__);
                    goto cleanup;
                }
                ctx->out_phandle[output_id][plane_id] = shared_pool;
            } else {
                ERROR_PRINT ("ERROR:: output_id = %d shared output buffer size mismatch\n", output_id);
                goto cleanup;
            }
        } else {
          ERROR_PRINT ("ERROR:: Shared Output buffer pool create failed. Unable to join sessions\n");
          goto cleanup;
        }
      }
    }/* plane_id */
  }/* output_id */

  //Allocate memory for filtercoeffs and get addr HfltCoeff & VfltCoeff
  for (output_id = 0; output_id < max_outputs; output_id++) {
    b_size    = sizeof(ctx->FilterCoeffs[output_id].HfltCoeff);
    bo_handle = xma_plg_buffer_alloc(xma_session, b_size, false, &ret);
    if (ret == XMA_SUCCESS) {
        ctx->HfltCoeff_Buffer[output_id] = bo_handle;
        DEBUG_PRINT ("HfltCoeff[%d] : paddr = %p, bo_handle = %u, size = %lu",
                     output_id,
                     (void*)ctx->HfltCoeff_Buffer[output_id].paddr,
                     ctx->HfltCoeff_Buffer[output_id].data, b_size);
    } else {
        ERROR_PRINT("H_FilterCoeff Buffer Allocation Failed");
        goto cleanup;
    }
    //VfltCoeff
    b_size    = sizeof(ctx->FilterCoeffs[output_id].VfltCoeff);
    bo_handle = xma_plg_buffer_alloc(xma_session, b_size, false, &ret);
    if (ret == XMA_SUCCESS) {
        ctx->VfltCoeff_Buffer[output_id] = bo_handle;
        DEBUG_PRINT ("VfltCoeff[%d] : paddr = %p, bo_handle = %u, size = %lu",
                     output_id,
                     (void*)ctx->VfltCoeff_Buffer[output_id].paddr,
                     ctx->VfltCoeff_Buffer[output_id],
                     b_size);
    } else {
        ERROR_PRINT("V_FilterCoeff Buffer Allocation Failed");
        goto cleanup;
    }

    //Allocate device buffer for DDR Register Descriptor
    b_size = sizeof(*ctx->desc);
    bo_handle = xma_plg_buffer_alloc(xma_session, b_size, false, &ret);
    if (ret == XMA_SUCCESS) {
      ctx->desc_buffer[output_id] = bo_handle;
    } else {
      ERROR_PRINT("Command Block Device Buffer Allocation Failed");
      goto cleanup;
    }
  }

  //Allocate HOST memory for DDR Register Descriptor Context
  ctx->desc = (XV_MULTISCALER_DESCRIPTOR *)calloc(max_outputs, sizeof(*ctx->desc));
  if(!ctx->desc) {
    ERROR_PRINT("HW Descriptor Host Memory Allocation Failed");
    goto cleanup;
  }

  return XMA_SUCCESS;
cleanup:
  for (output_id = 0; output_id < max_outputs; output_id++) {
    //@TODO add and use pool sharing API in xvbm
    if (!ctx->session_mix_rate) {
      for (plane_id = 0; plane_id < get_num_video_planes(session->props.output[output_id].format); plane_id++) {
        if (ctx->out_phandle[output_id][plane_id]) {
            xvbm_buffer_pool_destroy(ctx->out_phandle[output_id][plane_id]);
        }
      }/* plane_id */
    }

    if(ctx->HfltCoeff_Buffer[output_id].data) {
      xma_plg_buffer_free(xma_session, ctx->HfltCoeff_Buffer[output_id]);
    }
    if (ctx->VfltCoeff_Buffer[output_id].data) {
      xma_plg_buffer_free(xma_session, ctx->VfltCoeff_Buffer[output_id]);
    }

    if(ctx->desc_buffer[output_id].data) {
      xma_plg_buffer_free(xma_session, ctx->desc_buffer[output_id]);
     }
  }/* output_id */

  if (ctx->desc)
    free(ctx->desc);

  return XMA_ERROR;
}

#ifdef DEBUG_MULTISCALE
static void
print_coef(int16_t dest_filt[64][12])
{
  int i=0, j=0;
  for ( i=0; i<64; i++) {
    for ( j=0; j<12; j++) {
      fprintf (stderr, "%d ", dest_filt[i][j]);
    }
    fprintf (stderr, "\n");
  }
}
#endif

void print_desc_config(XmaScalerSession *session)
{
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  int output_id, i;
  uint32_t value;

  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "--------- HW REG Configuration ---------");
  for (i=XV_MULTI_SCALER_CTRL_ADDR_NUM_OUTS_DATA; i<XV_MULTI_SCALER_CTRL_REGMAP_SIZE; i+=4) {
      //value = *(uint32_t *)&ctx->hw_reg[i];
      memcpy(&value, &ctx->hw_reg[i], sizeof(uint32_t));
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Reg Addr 0x%02x = 0x%x\n", i, value);
  }
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "----------------------------------------");

  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "--------- Descriptor Block Configuration ---------");
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Desc block start addr : %p\n", (void *)ctx->desc_buffer[0].paddr);
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Num output channels   : %d\n", ctx->num_outs);

  for (output_id = 0; output_id < ctx->num_outs; output_id++) {
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "--------- Channel-id %d Start---------", output_id);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Width_In            : %4d",ctx->desc[output_id].widthIn);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Width_Out           : %4d",ctx->desc[output_id].widthOut);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Height_In           : %4d",ctx->desc[output_id].heightIn);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Height_Out          : %4d",ctx->desc[output_id].heightOut);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "StrideIn            : %4d",ctx->desc[output_id].strideIn);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "StrideOut           : %4d",ctx->desc[output_id].strideOut);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "PixelFmt_In         : %4d",ctx->desc[output_id].inPixelFmt);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "PixelFmt_Out        : %4d",ctx->desc[output_id].outPixelFmt);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "PixelRate           : %4d",ctx->desc[output_id].pixelRate);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "LineRate            : %4d",ctx->desc[output_id].lineRate);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "HorzFilterCoeffAddr : %p", (void *)ctx->desc[output_id].hfltCoeffAddr);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "VertFilterCoeffAddr : %p", (void *)ctx->desc[output_id].vfltCoeffAddr);
      char msg[256];
      strcpy (msg, "SrcImgAddr(Planes)  : ");
      for (i = 0; i < get_num_video_planes(session->props.output[output_id].format); i++) {
          char msg2[256];
          sprintf(msg2, "%s[%d] = %p%s", msg, i, (void *)ctx->desc[output_id].srcImgBuf[i], (i < get_num_video_planes(session->props.output[output_id].format) - 1) ? ", " : "");
          strcpy (msg, msg2);
      }
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, msg);
      strcpy (msg, "DstImgAddr(Planes)  : ");
      for (i = 0; i < get_num_video_planes(session->props.output[output_id].format); i++) {
          char msg2[256];
          sprintf(msg2, "%s[%d] = %p%s", msg, i, (void *)ctx->desc[output_id].dstImgBuf[i], (i < get_num_video_planes(session->props.output[output_id].format) - 1) ? ", " : "");
          strcpy (msg, msg2);
      }
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, msg);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "nextAddr            : %p",(void *)ctx->desc[output_id].nxtaddr);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "--------- Channel-id %d End---------", output_id);
  }
}

/*****************************************************************************
 * write device kernel context memory with register updates
*****************************************************************************/
static void write_desc_data_to_device(XmaScalerSession *session)
{
  XmaSession xma_session = session->base;
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;

  int output_id;
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);

  for (output_id = 0; output_id < max_outputs ; output_id++) {
    //copy host config data to allocated device buffer
    memcpy(ctx->desc_buffer[output_id].data,
           &ctx->desc[output_id],
           ctx->desc_buffer[output_id].size);

    //send config data to device
    xma_plg_buffer_write(xma_session,
                         ctx->desc_buffer[output_id],
                         ctx->desc_buffer[output_id].size, 0);
  }
  //set device ddr start address for desc data in hw_reg
  memcpy((ctx->hw_reg + XV_MULTI_SCALER_CTRL_ADDR_START_ADDR_DATA), 
         &(ctx->desc_buffer[0].paddr),
         sizeof(uint64_t));
}

static int32_t write_registers(XmaScalerSession *session)
{
  XmaSession xma_session = session->base;
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  uint32_t value;
  int output_id;
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);

  /* write num outputs */
  value = ctx->num_outs;
  memcpy((ctx->hw_reg + XV_MULTI_SCALER_CTRL_ADDR_NUM_OUTS_DATA), &value, sizeof(value));

  for (output_id = 0; output_id < max_outputs ; output_id++) {
    /*in_height*/
    ctx->desc[output_id].heightIn = ctx->in_height[output_id];

    /*in_width*/
    ctx->desc[output_id].widthIn = ctx->in_width[output_id];

    /*out_height*/
    ctx->desc[output_id].heightOut = ctx->out_height[output_id];

    /*out_width*/
    ctx->desc[output_id].widthOut = ctx->out_width[output_id];

    /*in_format*/
    ctx->desc[output_id].inPixelFmt = ctx->in_format[output_id];

    /*out_format*/
    ctx->desc[output_id].outPixelFmt = ctx->out_format[output_id];

    /*pixel_rate*/
    ctx->desc[output_id].pixelRate = ctx->pixel_rate[output_id];

    /*line_rate*/
    ctx->desc[output_id].lineRate = ctx->line_rate[output_id];

    /*in_stride*/
    ctx->desc[output_id].strideIn = ctx->in_stride[output_id];

    /*out_stride*/
    ctx->desc[output_id].strideOut = ctx->out_stride[output_id];

    /*Filter coefficients*/
    ctx->desc[output_id].hfltCoeffAddr = ctx->HfltCoeff_Buffer[output_id].paddr;
    ctx->desc[output_id].vfltCoeffAddr = ctx->VfltCoeff_Buffer[output_id].paddr;

    //copy Horz Filter Coeffs to allocated buffer
    memcpy(ctx->HfltCoeff_Buffer[output_id].data,
           ctx->FilterCoeffs[output_id].HfltCoeff,
           ctx->HfltCoeff_Buffer[output_id].size);

    //send Horz Filter Data to device
    xma_plg_buffer_write(xma_session,
                         ctx->HfltCoeff_Buffer[output_id],
                         ctx->HfltCoeff_Buffer[output_id].size, 0);

    //copy Vert Filter Coeffs to allocated buffer
    memcpy(ctx->VfltCoeff_Buffer[output_id].data,
           ctx->FilterCoeffs[output_id].VfltCoeff,
           ctx->VfltCoeff_Buffer[output_id].size);

    //send Vert Filter Data to device
    xma_plg_buffer_write(xma_session,
                         ctx->VfltCoeff_Buffer[output_id],
                         ctx->VfltCoeff_Buffer[output_id].size, 0);

    //set address of next block, in device memory
    if (output_id < (max_outputs-1)) {
        ctx->desc[output_id].nxtaddr = ctx->desc_buffer[output_id+1].paddr;
    } else {
        ctx->desc[output_id].nxtaddr = 0;
    }
  }
  return XMA_SUCCESS;
}

/* Multi Scaler Initialization
 * Uses session properties to create buffers, selecting filter coefficients and
 * write one time kernel configuration registers/buffers
 */
static int32_t
xlnx_multi_scaler_init(XmaScalerSession *session)
{
  openlog ("XMA_Scaler", LOG_PID, LOG_USER);
  assert(session != NULL);
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  int output_id = 0;
  int max_outputs;
  //XmaParameter* param;  
  int32_t xma_ret = XMA_SUCCESS;

  ctx->enable_pipeline  = 0;
  ctx->session_mix_rate = NULL;
  syslog(LOG_DEBUG, "xma_scaler_handle = %p\n", ctx);
  clock_gettime (CLOCK_REALTIME, &ctx->latency);
  ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
  syslog(LOG_DEBUG, "%s : %p :  xma_start at %lld \n", __func__, ctx, ctx->time_taken);

#ifdef DUMP_INPUT_FRAMES
  infp = fopen ("xma_inframes.yuv", "w+");
#endif

  //extract user extended property params
  get_user_params(session);
  xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "MultiScaler Pipeline Mode: %s", ((ctx->enable_pipeline)  ? "Enabled" : "Disabled"));
  if (ctx->session_mix_rate)
    xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "MultiScaler MixRate  Mode: Enabled");

  ctx->num_outs = session->props.num_outputs;
  max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);
  ctx->current_pipe = 0;
  ctx->first_frame = 0;
  ctx->recv_frame_cnt = 0;
  ctx->sent_frame_cnt = 0;
  ctx->s_idx = 0;
  ctx->r_idx = 0;

  if (ctx->num_outs > MAX_OUTPUTS) {
     ERROR_PRINT("Number of outputs programmed %d, exceeds Maximum supported outputs %d.", ctx->num_outs, MAX_OUTPUTS);
     return XMA_ERROR;
  }
  else if (ctx->num_outs <= 0) {
     ERROR_PRINT("Number of outputs programmed %d, is incorrect. Need to mention atleast 1 to get a scaler output.", ctx->num_outs, MAX_OUTPUTS);
     return XMA_ERROR;
  }

  if ((session->props.input.width % MULTISCALER_PPC) > 0) {
     ERROR_PRINT("in_width=%d is not supported as it is not a multiple of %d.\n",session->props.input.width,MULTISCALER_PPC );
     return XMA_ERROR;
  }
  
  if ((session->props.input.height % MULTISCALER_PPC) > 0) {
     ERROR_PRINT("in_height=%d is not supported as it is not a multiple of %d.\n",session->props.input.height,MULTISCALER_PPC );
     return XMA_ERROR;
  }

  for (output_id=0; output_id < max_outputs; output_id++) {
    if (output_id==0) {
      ctx->in_height[output_id] = session->props.input.height;
      ctx->in_width[output_id]  = session->props.input.width;
      ctx->in_format[output_id] = get_multiscaler_ip_format(session->props.input.format);
      ctx->in_stride[output_id] = MULTISCALER_ALIGN(session->props.input.stride, VCU_WIDTH_ALIGN);
    } else {
      /* assign input parameters with previous channel output paramters */
      ctx->in_height[output_id] = session->props.output[output_id-1].height;
      ctx->in_width[output_id]  = session->props.output[output_id-1].width;
      ctx->in_format[output_id] = get_multiscaler_ip_format(session->props.output[output_id-1].format);
      ctx->in_stride[output_id] = MULTISCALER_ALIGN(session->props.output[output_id-1].stride, VCU_WIDTH_ALIGN);
    }
    ctx->out_height[output_id] = session->props.output[output_id].height;
    ctx->out_width[output_id]  = session->props.output[output_id].width;
    ctx->out_format[output_id] = get_multiscaler_ip_format(session->props.output[output_id].format);
    ctx->out_stride[output_id] = MULTISCALER_ALIGN(session->props.output[output_id].stride, VCU_WIDTH_ALIGN);

    //check if resolution is within the limits (landscape or portrait mode)
    if ((ctx->in_width[output_id]<=0)   || (ctx->in_width[output_id]>MAX_WIDTH)   ||
        (ctx->in_height[output_id]<=0)  || (ctx->in_height[output_id]>MAX_WIDTH)  ||
        (ctx->out_width[output_id]<=0)  || (ctx->out_width[output_id]>MAX_WIDTH)  ||
        (ctx->out_height[output_id]<=0) || (ctx->out_height[output_id]>MAX_WIDTH) ||
        ((ctx->in_width[output_id]  * ctx->in_height[output_id])  > MAX_PIXELS)   ||
        ((ctx->out_width[output_id] * ctx->out_height[output_id]) > MAX_PIXELS)) {
         ERROR_PRINT("Output %d (%dx%d to %dx%d): Maximum supported "
             "resolution is %4dx%4d (%4dx%4d) and Minimum is 256x144.\n",
             output_id,ctx->in_width[output_id],ctx->in_height[output_id],ctx->out_width[output_id],
             ctx->out_height[output_id],MAX_WIDTH,MAX_HEIGHT,MAX_HEIGHT,MAX_WIDTH);
         return XMA_ERROR;
    }
    
    if ((ctx->out_width[output_id] % MULTISCALER_PPC) > 0) {
        ERROR_PRINT("out_%d_width=%d is not supported as it is not a multiple of %d.\n",output_id+1, ctx->out_width[output_id],MULTISCALER_PPC );
        return XMA_ERROR;
    }

    if ((ctx->out_height[output_id] % MULTISCALER_PPC) > 0) {
        ERROR_PRINT("out_%d_height=%d is not supported as it is not a multiple of %d.\n",output_id+1, ctx->out_height[output_id],MULTISCALER_PPC );
        return XMA_ERROR;
    }

    ctx->pixel_rate[output_id] = (uint32_t)((float)((ctx->in_width[output_id]*
        STEP_PRECISION)+(ctx->out_width[output_id]/2))/(float)ctx->out_width[output_id]);
    ctx->line_rate[output_id] = (uint32_t)((float)((ctx->in_height[output_id]*
        STEP_PRECISION)+(ctx->out_height[output_id]/2))/(float)ctx->out_height[output_id]);
  }

  for (output_id = 0; output_id < max_outputs; output_id++) {
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "----------- Channel [%d] Params START -----------", output_id);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Input : width = %u, height = %u, multiscale_fmt = %d (xma_fmt = %d), stride = %d",
          ctx->in_width[output_id], ctx->in_height[output_id], ctx->in_format[output_id],
          output_id == 0 ? session->props.input.format : session->props.output[output_id-1].format,
          ctx->in_stride[output_id]);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Output : width = %u, height = %u, multiscale_fmt = %d (xma_fmt = %d), stride = %d",
          ctx->out_width[output_id], ctx->out_height[output_id], ctx->out_format[output_id], session->props.output[output_id].format, ctx->out_stride[output_id]);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Channel pixel_rate = %d, linerate = %d", ctx->pixel_rate[output_id], ctx->line_rate[output_id]);
      xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "----------- Channel [%d] Params END -----------", output_id);
  }

  /* prepare filter coefficients */
  xma_ret = xlnx_multi_scaler_prepare_filter_tables (session);
  if (xma_ret != XMA_SUCCESS) {
    ERROR_PRINT ("failed to prepare filter tables...");
    return xma_ret;
  }

  /* Allocate buffers for input and output channels */
  xma_ret = multi_scaler_allocate_buffers (session);
  if (xma_ret != XMA_SUCCESS) {
    ERROR_PRINT ("failed to allocate buffers...");
    return xma_ret;
  }

  /* write to context info to registers */
  xma_ret = write_registers(session);
  if (xma_ret != XMA_SUCCESS) {
    ERROR_PRINT ("failed to write registers...");
    return xma_ret;
  }
  ctx->current_pipe = 0;
#ifdef MEASURE_TIME
  ctx->send_count = 0;
  ctx->recv_count = 0;
  ctx->send_func_time = 0;
  ctx->recv_func_time = 0;
  ctx->send_xrt_time = 0;
  ctx->recv_xrt_time = 0;
#endif
  ctx->pool_extended = false;

  ctx->frame_sent = 0;
  ctx->frame_recv = 0;
  clock_gettime (CLOCK_REALTIME, &ctx->latency);
  ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
  syslog(LOG_DEBUG, "%s : %p :  xma_end at %lld \n", __func__, ctx, ctx->time_taken);

  return XMA_SUCCESS;
}

static int get_raw_host_frame(MultiScalerContext *ctx, XmaFrame *frame)
{
  uint8_t *src, *dst;
  size_t size_y, size_uv;
  int16_t src_stride, dst_stride;
  int16_t a_height;
  int ret = 0;
  int i;

  ctx->in_bhandle[ctx->s_idx] = xvbm_buffer_pool_entry_alloc (ctx->in_phandle);
  uint8_t *dst_start_ptr = (uint8_t *)xvbm_buffer_get_host_ptr(ctx->in_bhandle[ctx->s_idx]);

  a_height = ALIGN(ctx->in_height[0], VCU_HEIGHT_ALIGN);
  size_y   = ((size_t)ctx->in_stride[0] * a_height);
  size_uv  = size_y >> 1;
  dst = dst_start_ptr;

  if ((frame->frame_props.linesize[0] != (int32_t)ctx->in_stride[0]) ||
      (frame->frame_props.height      != a_height))
  {
      //PLANE_Y
      src        = (uint8_t *)frame->data[0].buffer;
      src_stride = frame->frame_props.linesize[0];
      dst_stride = ctx->in_stride[0];
      for(i = 0; i < frame->frame_props.height; ++i) {
          memcpy(dst, src, frame->frame_props.width);
          src += src_stride;
          dst += dst_stride;
      }
      //PLANE_UV
      src = (uint8_t *)frame->data[1].buffer;
      dst = dst_start_ptr;
      dst += size_y;
      src_stride = frame->frame_props.linesize[1];
      dst_stride = ctx->in_stride[0];
      for(i = 0; i < frame->frame_props.height/2; ++i) {
          memcpy(dst, src, frame->frame_props.width);
          src += src_stride;
          dst += dst_stride;
      }
  } else {
      memcpy(dst, frame->data[0].buffer, size_y);
      dst += size_y;
      memcpy(dst, frame->data[1].buffer, size_uv);
  }
  ret = xvbm_buffer_write(ctx->in_bhandle[ctx->s_idx],
                          dst_start_ptr,
                          (size_y+size_uv), 0);
  return (ret);
}

/* Writes input buffer at channel-0 */
static int32_t
prep_and_write_input_buffer (XmaScalerSession *session, int32_t buf_idx, XmaFrame *frame)
{
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  uint64_t paddr;
  uint64_t offset;

  (void)buf_idx; //unused param
  if (session->props.input.format == XMA_VCU_NV12_FMT_TYPE) {
    if (frame->data[0].buffer_type == XMA_DEVICE_BUFFER_TYPE)	
       ctx->in_bhandle[ctx->s_idx] = (XvbmBufferHandle)(frame->data[0].buffer);
    else {
        if(get_raw_host_frame(ctx, frame)) {
          ERROR_PRINT("host buffer write failed\n");
          return XMA_ERROR;
        }
    }
   
    if (ctx->in_bhandle[ctx->s_idx]) {
      //Extend Input Pool, if needed
      if (!ctx->pool_extended) {
        if (ctx->latency_logging) {
          clock_gettime (CLOCK_REALTIME, &ctx->latency);
          ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
          syslog(LOG_DEBUG, "%s : xma_extend start at : %lld\n", __func__, ctx->time_taken);
        }

        uint32_t num = xvbm_buffer_pool_num_buffers_get(ctx->in_bhandle[ctx->s_idx]);
        xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Num of buffers allocated in previous component = %d\n", num);
        /* Depending on available XRT buffer, pool can be extended more (currently 4) */

        uint32_t extend_cnt_needed = 0;
        if (ctx->enable_pipeline)
          extend_cnt_needed = 2;
  
        uint32_t cnt = xvbm_buffer_pool_extend(ctx->in_bhandle[ctx->s_idx],
                                               MAX_PIPELINE_BUFFERS + extend_cnt_needed);
        if (cnt == num + MAX_PIPELINE_BUFFERS + extend_cnt_needed) {
          ctx->pool_extended = true;
          xma_logmsg(XMA_DEBUG_LOG, XMA_MULTISCALER, "Extended previous component's output pool to %d buffers\n", cnt);
        } else {
          ERROR_PRINT("Extention failed");
          return XMA_ERROR;
        }

        if (ctx->latency_logging) {
          clock_gettime (CLOCK_REALTIME, &ctx->latency);
          ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
          syslog(LOG_DEBUG, "%s : xma_extend end at : %lld\n", __func__, ctx->time_taken);
        }
      }
      XVBM_BUFF_PR("\tMS Recv buffer from DEC =%p ID = %d\n",
                    ctx->in_bhandle[ctx->s_idx],
                    xvbm_buffer_get_id(ctx->in_bhandle[ctx->s_idx]));
      
      paddr = xvbm_buffer_get_paddr(ctx->in_bhandle[ctx->s_idx]);
      ctx->desc[0].srcImgBuf[0] = paddr;

      /* prep_write plane-1 with offset stride * elevation */
      offset = ctx->in_stride[0] * ALIGN(ctx->in_height[0], VCU_HEIGHT_ALIGN);
      paddr += offset;
      ctx->desc[0].srcImgBuf[1] = paddr;
    } else {
        ERROR_PRINT ("invalid input buffer handle in scaler\n");
        return XMA_ERROR;
    }
  } else {
      /* TODO : Not supported yet */
      ERROR_PRINT ("Input format %d not supported. Must be XMA_VCU_NV12_FMT_TYPE\n", session->props.input.format);
      return XMA_ERROR;
  }

  return XMA_SUCCESS;
}

/* do prep_write for all input & output channels except channel-0 input */
static int32_t
prepare_inout_buffers (XmaScalerSession *session, int32_t buf_idx)
{
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  int32_t output_id;
  int32_t plane_id = 0;
  uint64_t value = 0;
  int32_t max_try = 0;
  uint64_t paddr, offset;
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);
  XvbmBufferHandle b_handle;

  (void)buf_idx; //unused param
  for (output_id = 0; output_id < max_outputs; output_id++) {
    if (session->props.output[output_id].format == XMA_VCU_NV12_FMT_TYPE) {
      uint64_t offset = 0;
      do {
        b_handle = xvbm_buffer_pool_entry_alloc(ctx->out_phandle[output_id][0]);
        if (b_handle != NULL) {
          XVBM_BUFF_PR("\tMS: got free buffer from pool %p id = %d\n",
                       b_handle, xvbm_buffer_get_id(b_handle));
          max_try = 0;
          break;
        } else {
          max_try++;
          if (max_try >=100) {
            ERROR_PRINT ("xvbm_buffer_pool_entry_alloc failed forever\n");
            goto TRY_AGAIN;
          }
          ERROR_PRINT ("Unable to get free buffer for encoder for channel id = %d, retrying..\n",
                       output_id);
          usleep(2000);
        }
      } while (b_handle == NULL);

      /* Save buffer handler to get same in receive call */
      ctx->out_bhandle[output_id][ctx->s_idx][0] = b_handle;
      XVBM_BUFF_PR("MS adding buffer for output_id = %d, ctx->s_idx = %d\n", output_id,ctx->s_idx);
      paddr = xvbm_buffer_get_paddr(b_handle);
      ctx->desc[output_id].dstImgBuf[0] = paddr;
      
      if (output_id < (max_outputs-1)) //Since Input is cascaded  loop starts with 1
        /* prepare input register write at 'output_id+1' (in[1..7] = out[0..6]*/
        ctx->desc[output_id+1].srcImgBuf[0] = paddr;

      offset = ctx->out_stride[output_id] * ALIGN(ctx->out_height[output_id], VCU_HEIGHT_ALIGN);
      paddr += offset;
      ctx->desc[output_id].dstImgBuf[1] = paddr;
      if (output_id < (max_outputs-1))
        ctx->desc[output_id+1].srcImgBuf[1] = paddr;
    } else {
        for (plane_id = 0; plane_id < get_num_video_planes(session->props.output[output_id].format); plane_id++) {
          do {
            b_handle = xvbm_buffer_pool_entry_alloc(ctx->out_phandle[output_id][plane_id]);
            if (b_handle != NULL) {
              XVBM_BUFF_PR("\tMS: got free buffer from pool %p id = %d\n",
                  b_handle, xvbm_buffer_get_id(b_handle));
              max_try = 0;
              break;
            } else {
               max_try++;
              if (max_try >=100) {
                ERROR_PRINT ("xvbm_buffer_pool_entry_alloc failed forever\n");
                goto TRY_AGAIN;
              }
              ERROR_PRINT ("Unable to get free buffer for encoder for channel id = %d, retrying..\n",
                          output_id);
              usleep(2000);
            }
          } while (b_handle == NULL);

          /*
           * Save buffer handler to get same in receive call
           * s_idx should be equal to r_idx in recv
           */
          ctx->out_bhandle[output_id][ctx->s_idx][plane_id] = b_handle;
          paddr = xvbm_buffer_get_paddr(b_handle);
          ctx->desc[output_id].dstImgBuf[plane_id] = paddr;

          if (output_id < (max_outputs-1))
            /* prepare input register write at 'output_id+1' */
	    ctx->desc[output_id+1].srcImgBuf[plane_id] = paddr;
        }//for (plane_id)
    } //if (session->props.output[output_id].format == XMA_VCU_NV12_FMT_TYPE)
  }// for (output_id
  write_desc_data_to_device(session);
  print_desc_config(session);
  return XMA_SUCCESS;
TRY_AGAIN:
  /* Clean up allocated buffers */
  for (int output = 0; output <= output_id; output++) {
    for (int plane = 0; plane < get_num_video_planes(session->props.output[output].format); plane++) {
      XvbmBufferHandle b_handle = ctx->out_bhandle[output][ctx->s_idx][plane];
      if (b_handle != NULL) {
        xvbm_buffer_pool_entry_free(b_handle);
        ctx->out_bhandle[output][ctx->s_idx][plane] = NULL;
      }
    }
  }
  return XMA_TRY_AGAIN;
}

static int32_t
xlnx_multi_scaler_flush_frame (XmaScalerSession *session)
{
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  XmaSession xma_session = session->base;
  int32_t ret=0;

  if ((ctx->recv_frame_cnt - ctx->sent_frame_cnt) > 1) {
    xma_plg_schedule_work_item(xma_session, ctx->hw_reg, XV_MULTI_SCALER_CTRL_REGMAP_SIZE, &ret);
    return XMA_FLUSH_AGAIN;
  } else if ((ctx->recv_frame_cnt - ctx->sent_frame_cnt) == 1) {
    return XMA_FLUSH_AGAIN;
  } else {
    DEBUG_PRINT ("return EOS. recv_frame_cnt = %d and sent_frame_cnt = %d\n", ctx->recv_frame_cnt, ctx->sent_frame_cnt);
    return XMA_EOS;
  }
}

static int32_t
xlnx_multi_scaler_send_frame(XmaScalerSession *session, XmaFrame *frame)
{
  int ret;
  assert(session != NULL);
  assert(frame != NULL);
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  XmaSession xma_session = session->base;
  int buf_idx = ctx->current_pipe;
  int32_t xma_ret = XMA_SUCCESS;
#ifdef MEASURE_TIME
  struct timespec start, stop;
  struct timespec rstart, rstop;
  ctx->send_count++;
  clock_gettime (CLOCK_REALTIME, &start);
#endif
  DEBUG_PRINT ("enter");

  if (frame->data[0].buffer == NULL) {
    /* Null buffer indicates EOS, flush pending data in the pipe */
    return xlnx_multi_scaler_flush_frame (session);
  }

#ifdef DUMP_INPUT_FRAMES
  {
    int written = fwrite (frame->data[0].buffer, 1, ctx->in_stride[0] * ctx->in_height[0], infp);
    fprintf (stderr, "written inframe[0] size = %d\n", written);
    written = fwrite (frame->data[1].buffer, 1, (ctx->in_stride[0] * ctx->in_height[0]) >> 1, infp);
    fprintf (stderr, "written inframe[1] size = %d\n", written);
  }
#endif
  // TODO: why to use s_idx, instead use buf_idx
  ctx->pts[ctx->s_idx] = frame->pts;
  ctx->is_idr[ctx->s_idx] = frame->is_idr;
  ctx->time_base[ctx->s_idx] = frame->time_base;
  ctx->frame_rate[ctx->s_idx] = frame->frame_rate;
  ctx->recv_frame_cnt++;

  DEBUG_PRINT ("Received frame number %lu : pts = %lu, buffer index in pool = %d",
      ctx->recv_frame_cnt, ctx->pts[ctx->s_idx], ctx->s_idx);

  if (ctx->latency_logging) {
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->frame_sent++;
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog(LOG_DEBUG, "%s : %p : xma_scaler_frame_sent %lld : %lld\n", __func__, ctx, ctx->frame_sent, ctx->time_taken);
  }
      
  if (ctx->first_frame == 0) {
    /* write input frame at index 0 */
    ret =  prep_and_write_input_buffer(session, buf_idx, frame);
    ret |= prepare_inout_buffers(session, buf_idx);
    if (ret != XMA_SUCCESS)
      return ret;
  } else {
    if (ctx->enable_pipeline) {
       /* schedule a request to XRT. This will execute for previous buffer */
       XmaCUCmdObj cu_cmd = xma_plg_schedule_work_item(xma_session, ctx->hw_reg,XV_MULTI_SCALER_CTRL_REGMAP_SIZE, &xma_ret);
       (void)cu_cmd; //currently unused
       if (xma_ret != XMA_SUCCESS) {
         ERROR_PRINT ("failed schedule request to XRT...val = %d", xma_ret);
         return xma_ret;
       }
     }

    /* prepare & write input buffer at channel-0 */
    ret  = prep_and_write_input_buffer(session, buf_idx, frame);
    /* prepare input & output registers write at all channels except input channel-0 */
    ret |= prepare_inout_buffers(session, buf_idx);
    if (ret != XMA_SUCCESS)
      return ret;
  }

  if (!ctx->enable_pipeline) {
    XmaCUCmdObj cu_cmd  = xma_plg_schedule_work_item(xma_session, ctx->hw_reg, XV_MULTI_SCALER_CTRL_REGMAP_SIZE, &xma_ret);
    (void)cu_cmd; //currently unused
    if (xma_ret != XMA_SUCCESS) {
      ERROR_PRINT ("failed schedule request to XRT...val = %d", xma_ret);
      return xma_ret;
    }
  }
  
  ctx->s_idx = (ctx->s_idx + 1) % MAX_OUTPOOL_BUFFERS;
  ctx->current_pipe = (ctx->current_pipe + 1) % MAX_OUTPOOL_BUFFERS;

  DEBUG_PRINT ("current pipe = %d", ctx->current_pipe);
  if (ctx->enable_pipeline) {
     if (ctx->first_frame < MAX_PIPELINE_BUFFERS) {
       ctx->first_frame++;
       /* needs more input frames to support pipelining */
       DEBUG_PRINT ("send more data due to pipelining");
#ifdef MEASURE_TIME
       clock_gettime (CLOCK_REALTIME, &stop);
       ctx->send_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 + (stop.tv_nsec - start.tv_nsec) / 1e3);
      if (ctx->send_count ==  MAX_COUNT_TIME) {
         fprintf (stderr, "Scaler send [%p]: %lld\n", ctx, ctx->send_func_time/ctx->send_count);
         ctx->send_func_time = ctx->send_count = 0;
       }
#endif
       return XMA_SEND_MORE_DATA;
     }
  }
#ifdef MEASURE_TIME
  clock_gettime (CLOCK_REALTIME, &stop);
  ctx->send_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 + (stop.tv_nsec - start.tv_nsec) / 1e3);
  if (ctx->send_count ==  MAX_COUNT_TIME) {
    fprintf (stderr, "Scaler send [%p]: %lld\n", ctx, ctx->send_func_time/ctx->send_count);
    ctx->send_func_time = ctx->send_count = 0;
  }
#endif
  DEBUG_PRINT ("leave");
  return XMA_SUCCESS;
}

static int32_t
xlnx_multi_scaler_recv_frame_list(XmaScalerSession *session, XmaFrame **frame_list)
{
  assert(session != NULL);
  assert(frame_list[0] != NULL);
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  XmaSession xma_session = session->base;
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);
  int32_t buf_idx;
  int output_id;
  int32_t xma_ret = XMA_SUCCESS;
#ifdef MEASURE_TIME
  struct timespec start, stop;
  struct timespec rstart, rstop;
  ctx->recv_count++;
  clock_gettime (CLOCK_REALTIME, &start);
#endif
  DEBUG_PRINT ("enter");

  if (ctx->enable_pipeline) {
     if (ctx->first_frame < MAX_PIPELINE_BUFFERS) {
       xma_logmsg(XMA_WARNING_LOG, XMA_MULTISCALER, "Called receive frame before sending %d buffers and current buffered frames %d", 2, ctx->first_frame);
       return XMA_SUCCESS;
     }
  }
#ifdef MEASURE_TIME
  clock_gettime (CLOCK_REALTIME, &rstart);
#endif

  //Check if frame processing is complete (Check DONE bit)
  xma_ret = xma_plg_is_work_item_done(xma_session, 1000);
  if (xma_ret != XMA_SUCCESS) {
    ERROR_PRINT ("Scaler Stopped responding");
    return xma_ret;
  }

#ifdef MEASURE_TIME
  clock_gettime (CLOCK_REALTIME, &rstop);
  ctx->recv_xrt_time += ((rstop.tv_sec - rstart.tv_sec) * 1e6 + (rstop.tv_nsec - rstart.tv_nsec) / 1e3);
#endif

  if (ctx->latency_logging) {
    clock_gettime (CLOCK_REALTIME, &ctx->latency);
    ctx->frame_recv++;
    ctx->time_taken = (ctx->latency.tv_sec * 1e3) + (ctx->latency.tv_nsec / 1e6);
    syslog(LOG_DEBUG, "%s : %p : xma_scaler_frame_recv %lld : %lld\n", __func__, ctx, ctx->frame_recv, ctx->time_taken);
  }

  buf_idx = ctx->r_idx;

  for (output_id = 0; output_id < max_outputs; output_id++) {
    frame_list[output_id]->pts = ctx->pts[ctx->r_idx];
    frame_list[output_id]->is_idr = ctx->is_idr[ctx->r_idx];
    frame_list[output_id]->time_base = ctx->time_base[ctx->r_idx];
    frame_list[output_id]->frame_rate = ctx->frame_rate[ctx->r_idx];

      int32_t plane_id = 0;

      if (session->props.output[output_id].format == XMA_VCU_NV12_FMT_TYPE) {
          XvbmBufferHandle b_handle = ctx->out_bhandle[output_id][ctx->r_idx][plane_id];
          if (!b_handle) {
              ERROR_PRINT ("ERROR , no bhandle found\n");
              return XMA_ERROR;
          }

          if (frame_list[output_id]->data[0].buffer_type == XMA_DEVICE_BUFFER_TYPE) {
              frame_list[output_id]->data[plane_id].buffer = (void*)b_handle;
          } else {
              uint32_t size = ALIGN(ctx->out_width[output_id], VCU_WIDTH_ALIGN) * ALIGN(ctx->out_height[output_id], VCU_HEIGHT_ALIGN);
              uint8_t *hbuf;
              int32_t ret;

              hbuf = (uint8_t *)xvbm_buffer_get_host_ptr(b_handle);
              if (!hbuf) {
                  ERROR_PRINT ("Invalid host buffer\n");
                  return XMA_ERROR;
              }

              ret  = xvbm_buffer_read(b_handle, hbuf, (size+size/2), 0);
              if (ret) {
                ERROR_PRINT ("host buffer read failed\n");
                return XMA_ERROR;
              }

              memcpy(frame_list[output_id]->data[0].buffer, hbuf,        size);
              memcpy(frame_list[output_id]->data[1].buffer, (hbuf+size), size/2);
              xvbm_buffer_pool_entry_free (ctx->out_bhandle[output_id][ctx->r_idx][plane_id]);
          }

          XVBM_BUFF_PR("\tMS sending output buffer =%p ID = %d\n", b_handle, xvbm_buffer_get_id(b_handle));
          XVBM_BUFF_PR("MS REMOVE buffer for output_id = %d, ctx->r_idx = %d, plane_id = %d\n",
                output_id,ctx->r_idx, plane_id );

            ctx->out_bhandle[output_id][ctx->r_idx][plane_id] = NULL;
      } else {
        /* TODO: support something other than XMA_VCU_NV12_FMT_TYPE */
      }
  }
  
  ctx->sent_frame_cnt = ctx->sent_frame_cnt + 1;
  ctx->r_idx = (ctx->r_idx + 1) % MAX_OUTPOOL_BUFFERS;

  if (ctx->in_bhandle[buf_idx]) {
      XVBM_BUFF_PR("\tMS free input buffer =%p ID = %d\n",
                   ctx->in_bhandle[buf_idx],
      xvbm_buffer_get_id(ctx->in_bhandle[buf_idx]));
      xvbm_buffer_pool_entry_free(ctx->in_bhandle[buf_idx]);
  }

  DEBUG_PRINT ("read buffer at index %d and sent frame count is %lu", buf_idx, ctx->sent_frame_cnt);
#ifdef MEASURE_TIME
  clock_gettime (CLOCK_REALTIME, &stop);
  ctx->recv_func_time += ((stop.tv_sec - start.tv_sec) * 1e6 + (stop.tv_nsec - start.tv_nsec) / 1e3);
  if (ctx->recv_count ==  MAX_COUNT_TIME) {
    fprintf (stderr, "Scaler recv[%p] : %lld \t %lld\n", ctx, ctx->recv_func_time/ctx->recv_count, ctx->recv_xrt_time/ctx->recv_count);
    ctx->recv_func_time = ctx->recv_xrt_time = ctx->recv_count = 0;
  }
#endif
  DEBUG_PRINT ("leave");
  return XMA_SUCCESS;
}


static int32_t
xlnx_multi_scaler_close(XmaScalerSession *session)
{
  assert(session != NULL);
  XmaSession xma_session = session->base;
  MultiScalerContext *ctx = (MultiScalerContext*)session->base.plugin_data;
  int max_outputs = MIN(ctx->num_outs, MAX_OUTPUTS);
  int plane_id, output_id;
  DEBUG_PRINT ("enter");
#ifdef DUMP_INPUT_FRAMES
  fclose (infp);
#endif

  //release input buffer pool
  if (ctx->in_phandle)
    xvbm_buffer_pool_destroy(ctx->in_phandle);

  //release filter coeff and output buffers
  for (output_id = 0; output_id < max_outputs; output_id++) {
    xma_plg_buffer_free(xma_session, ctx->HfltCoeff_Buffer[output_id]);
    xma_plg_buffer_free(xma_session, ctx->VfltCoeff_Buffer[output_id]);
    xma_plg_buffer_free(xma_session, ctx->desc_buffer[output_id]);
    //@TODO add and use pool sharing API in xvbm
    if (!ctx->session_mix_rate) {
        for (plane_id = 0; plane_id < get_num_video_planes(session->props.output[output_id].format); plane_id++) {
            if (ctx->out_phandle[output_id][plane_id]) {
                xvbm_buffer_pool_destroy(ctx->out_phandle[output_id][plane_id]);
            }
        }/* plane_id */
    }
  }/* output_id */

  if(ctx->desc)
    free(ctx->desc);

  DEBUG_PRINT ("leave");
  closelog();
  return XMA_SUCCESS;
}

/*------------------------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------------------------*/
static int32_t xma_scaler_version(int32_t *main_version, int32_t *sub_version)
{
    *main_version = 2020;
    *sub_version = 1;

    return 0;
}

XmaScalerPlugin scaler_plugin = {
  .hwscaler_type    = XMA_POLYPHASE_SCALER_TYPE,
  .hwvendor_string  = "Xilinx",
  .input_format     = XMA_YUV420_FMT_TYPE,
  .output_format    = XMA_RGB888_FMT_TYPE,
  .bits_per_pixel   = 8,
  .plugin_data_size = sizeof(MultiScalerContext),
  .init             = xlnx_multi_scaler_init,
  .send_frame       = xlnx_multi_scaler_send_frame,
  .recv_frame_list  = xlnx_multi_scaler_recv_frame_list,
  .close            = xlnx_multi_scaler_close,
  .xma_version      = xma_scaler_version,
  .reserved         = {0}
};
