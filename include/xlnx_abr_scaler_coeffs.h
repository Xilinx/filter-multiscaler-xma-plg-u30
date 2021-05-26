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
#ifndef _XLNX_ABR_SCALER_COEFFS_H_
#define _XLNX_ABR_SCALER_COEFFS_H_

/**
 *  @file
 */
#define MIN(a,b)                ((a)<(b))?(a):(b)
#define MAX(a,b)                ((a)>(b))?(a):(b)
#define ABS(a)                  ((a)<0)? (-(a)): (a)
#define CLIP(a,b,c)             ((a)<(b))? (b):(((a)>(c)) ? (c) : (a))
#define ROUNDED_DIV(a,b)        (((a)>0 ? (a) + ((b)>>1) : (a) - ((b)>>1))/(b))
#define SWS_MAX_REDUCE_CUTOFF   0.000001
#define MAX_FILTER_SIZE         12
#define DMA_SIZE                128
#define MAX_OUTS                8
#define HSC_PHASES              64
#define VSC_PHASES              64
#define NR_PHASE_BITS           6
#define NR_PHASES               (1<<NR_PHASE_BITS)
#define HSC_TAPS                12
#define VSC_TAPS                12
#define STEP_PRECISION_SHIFT    16
#define STEP_PRECISION          (1<<STEP_PRECISION_SHIFT)

//#define COEFF_DUMP

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ScalerFilterCoeffs
{
    int16_t             HfltCoeff[HSC_PHASES][HSC_TAPS];	
    int16_t             VfltCoeff[VSC_PHASES][VSC_TAPS];
} ScalerFilterCoeffs;

int log2_val(unsigned int val)
{
      int cnt  = 0;
      while(val > 1)
      {
         val=val>>1;
         cnt++;
      }
      return cnt;
}

int feasibilityCheck(int src, int dst, int* filterSize)
{
    int sizeFactor = 4;  
    int xInc = (((int64_t)src << 16) + (dst >> 1)) / dst;
    if (xInc <= 1 << 16)
      *filterSize = 1 + sizeFactor;    // upscale
    else
      *filterSize = 1 + (sizeFactor * src + dst - 1) / dst; 
    
    if (*filterSize > MAX_FILTER_SIZE)
    {
       //printf("INFO: FilterSize %d for %d to %d is greater than maximum taps(%d)\n",  *filterSize, src, dst, MAX_FILTER_SIZE);		
       return XMA_ERROR;	  
    }
    return 0;	
}

void Generate_cardinal_cubic_spline(int src, int dst, int filterSize, int64_t B, int64_t C, int16_t* CCS_filtCoeff)
{
#ifdef COEFF_DUMP
    FILE *fp;
    char fname[512];
    sprintf(fname,"coeff_%dTO%d.csv",src,dst);
    fp=fopen(fname,"w");
    /*FILE *fph;
    sprintf(fname,"phase_%dTO%d_2Inc.txt",src,dst);	
    fph=fopen(fname,"w");*/
    //fprintf(fp,"src:%d => dst:%d\n\n",src,dst);
#endif
    int  filterAlign = 4,  one= (1<<14);
    int64_t *coeffFilter =NULL;
    int64_t *coeffFilter_reduced = NULL;	   
    int16_t *outFilter = NULL;
    int16_t *coeffFilter_normalized = NULL;
    int lumXInc      = (((int64_t)src << 16) + (dst >> 1)) / dst;
    int srt = src/dst;
    int lval = log2_val(srt);
    int th0 = 8;
    int lv0 = MIN(lval, th0);	   
    const int64_t fone = (int64_t)1 << (54-lv0);	 	   
    int64_t thr1 = ((int64_t)1 << 31);
    int64_t thr2 = ((int64_t)1<<54)/fone;	   
    int i, xInc, filter2Size, minFilterSize, outFilterSize;	   
    int num_phases = 64;
    int phase_set[64] ={0};
    int64_t xDstInSrc;
    int xx,j,ph1,diff;
    int64_t ld2=0, ph0=0, d=0, coeff=0, dd=0,ddd=0;  
    int phase_cnt =0, singleZP = 0,  min = 0, k=0;
    int64_t cutOff = 0.0;	       
    int64_t error = 0, sum = 0, v = 0;
    int intV =0, rot_idx = 0, col_idx=-1;
    int currCoeff=0, prevCoeff = -1;
    int c = 0, nzRow = 0, mi=0;
    int firstValidP = 0, re_phase_cnt = 0;
    int fstart_Idx=0, fend_Idx=0, half_Idx=0, middleIdx = 0;
  
	   
    xInc = lumXInc;  
    filterSize = MAX(filterSize, 1);
    coeffFilter = (int64_t*) calloc(num_phases*filterSize, sizeof(int64_t)); 
	
    xDstInSrc = xInc - (1<<16);	   
    diff =xDstInSrc;
    unsigned int PhaseH = 0, offset = 0, WriteLoc = 0, WriteLocNext = 0, ReadLoc=0, OutputWrite_En = 0;
	   
    int OutPixels = dst;
    int PixelRate = (int)((float)((src*STEP_PRECISION) + (dst/2))/(float)dst);
    
    // coefficient generation based on scaler IP
    for (i = 0; i < src; i++)
    {
        PhaseH = ((offset>>(STEP_PRECISION_SHIFT-NR_PHASE_BITS))) & (NR_PHASES-1);
        WriteLoc = WriteLocNext;
        
        if ((offset >> STEP_PRECISION_SHIFT) != 0)
        {
            // Take a new sample from input, but don't process anything
            ReadLoc++;
            offset = offset - (1<<STEP_PRECISION_SHIFT);
            OutputWrite_En   = 0;
            WriteLocNext = WriteLoc;
        }
        
        if (((offset >> STEP_PRECISION_SHIFT) == 0) && (WriteLoc<OutPixels))
        {
            // Produce a new output sample
            offset += PixelRate;
            OutputWrite_En   = 1;
            WriteLocNext = WriteLoc+1;
        }
        if (OutputWrite_En) {
            
            //printf("i %d out %d in %d phase %d\n", i, WriteLoc, ReadLoc, PhaseH);
            
            xDstInSrc = ReadLoc*(1<<17) + PhaseH*(1<<11);
            xx = ReadLoc - (filterSize - 2)/2;
            
            //printf("xx %d xDstInSrc %lld\n", xx, xDstInSrc);
            d = (ABS(((int64_t)xx * (1 << 17)) - xDstInSrc)) << 13;
            //printf("d %lld\n", d);

            //count number of phases used for this SR
            if (phase_set[PhaseH] == 0) phase_cnt+=1;
            
            //Filter coeff generation
            for (j = 0; j < filterSize; j++)
            {
                d = (ABS(((int64_t)xx * (1 << 17)) - xDstInSrc)) << 13;
                if (xInc > 1 << 16)
                {
                    //printf("xinc >\n");
                    d = (int64_t)(d *dst/ src);
                }
                
                if (d >= thr1) {
                    coeff = 0.0;
                } else {
                    dd  = (int64_t)(d  * d) >> 30;
                    ddd = (int64_t) (dd * d) >> 30;
                    if (d < 1 << 30)
                    {
                        coeff =  (12 * (1 << 24) -  9 * B - 6 * C) * ddd +
                        (-18 * (1 << 24) + 12 * B + 6 * C) *  dd +
                        (6 * (1 << 24) -  2 * B)         * (1 << 30);
                    }
                    else
                    {
                        coeff =  (-B -  6 * C) * ddd +
                        (6 * B + 30 * C) * dd  +
                        (-12 * B - 48 * C) * d   +
                        (8 * B + 24 * C) * (1 << 30);
                    }
                }
                
                coeff = coeff/thr2;
                coeffFilter[PhaseH * filterSize + j] = coeff;
                xx++;
            }
            if (phase_set[PhaseH] == 0) {
                phase_set[PhaseH] = 1;
            }
        }
    }
       
    //printf("filter size is %d\n", filterSize);
    coeffFilter_reduced = (int64_t*) calloc((num_phases*filterSize), sizeof(int64_t));
    memcpy(coeffFilter_reduced, coeffFilter, sizeof(int64_t)*num_phases*filterSize);
    outFilterSize = filterSize;
    outFilter = (int16_t*) calloc((num_phases*outFilterSize),sizeof(int16_t));
    coeffFilter_normalized = (int16_t*) calloc((num_phases*outFilterSize),sizeof(int16_t));
    
    /* normalize & store in outFilter */
    for ( i = 0; i < num_phases; i++) {
        error = 0;
        sum   = 0;
        
        for (j = 0; j < filterSize; j++) {
            sum += coeffFilter_reduced[i * filterSize + j];
        }
        sum = (sum + one / 2) / one;
        if (!sum) {
            sum = 1;
        }
        for (j = 0; j < outFilterSize; j++) {
            v = coeffFilter_reduced[i * filterSize + j] + error;
            intV  = ROUNDED_DIV(v, sum);
            coeffFilter_normalized[i * (outFilterSize) + j] = intV;
            coeffFilter_normalized[i * (outFilterSize) + j] = coeffFilter_normalized[i * (outFilterSize) + j]>>2; //added to negate double increment and match our precision
            error                                  = v - intV * sum;
        }
    }
    
    for (int p=0; p<num_phases; p++) {
        for (int t=0; t<filterSize; t++) {
            outFilter[p*filterSize + t] = coeffFilter_normalized[p*filterSize + t];
        }
    }
       
    /*incorporate filter less than 12 tap into a 12 tap*/
    fstart_Idx=0, fend_Idx=0, half_Idx=0;
    middleIdx = (MAX_FILTER_SIZE/2); //center location for 12 tap 
    half_Idx = (outFilterSize/2);
    if ( (outFilterSize - (half_Idx<<1)) ==0) //evenOdd
    {
        fstart_Idx = middleIdx - half_Idx;
        fend_Idx = middleIdx + half_Idx;
    }
    else
    {
        fstart_Idx = middleIdx - (half_Idx);
        fend_Idx = middleIdx + half_Idx + 1;
    }
    //printf("DEBUG: outFilterSize=%d fstart_Idx=%d fend_Idx=%d half_Idx=%d\n",outFilterSize,fstart_Idx,fend_Idx,half_Idx);
    
    for ( i = 0; i < num_phases; i++) {
        for ( j = 0; j < MAX_FILTER_SIZE; j++) {
            
            CCS_filtCoeff[i*MAX_FILTER_SIZE + j] = 0;
            if ((j >= fstart_Idx) && (j< fend_Idx))
                CCS_filtCoeff[i*MAX_FILTER_SIZE + j] = outFilter[i * (outFilterSize) + (j-fstart_Idx)];
            
        }
    }
	
#if 1
    /*Make sure filterCoeffs within a phase sum to 4096*/
    int ph_max_sum = 1<<MAX_FILTER_SIZE;
	int sumVal= 0 , maxIdx=0, maxVal=0, diffVal=0;
    for ( i = 0; i < num_phases; i++) {
        sumVal = 0;
        maxVal = 0;
        for ( j = 0; j < MAX_FILTER_SIZE; j++) {
            sumVal+=CCS_filtCoeff[i*MAX_FILTER_SIZE + j];
            if ( CCS_filtCoeff[i*MAX_FILTER_SIZE + j] > maxVal)
            {				  
                maxVal = CCS_filtCoeff[i*MAX_FILTER_SIZE + j];
                maxIdx = j;
            }             			  
        }
        diffVal = ph_max_sum - sumVal ;
        if (diffVal>0)
              CCS_filtCoeff[i*MAX_FILTER_SIZE + maxIdx] = CCS_filtCoeff[i*MAX_FILTER_SIZE + maxIdx]+diffVal;
    }   
#endif
	
           
#ifdef COEFF_DUMP                       
    fprintf(fp,"taps/phases, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12\n");
    for ( i = 0; i < num_phases; i++) {
        fprintf(fp,"%d, ", i+1);
        for ( j = 0; j < MAX_FILTER_SIZE; j++) {
            fprintf(fp,"%d,  ",CCS_filtCoeff[i*MAX_FILTER_SIZE + j]);
        }
        fprintf(fp,"\n");
    }      
#endif      

    free(coeffFilter);
    free(coeffFilter_reduced);
    free(outFilter);
    free(coeffFilter_normalized);
#ifdef COEFF_DUMP	
    fclose(fp);
#endif	
}


/**
 * @}
 */
 
#ifdef __cplusplus
}
#endif

#endif
