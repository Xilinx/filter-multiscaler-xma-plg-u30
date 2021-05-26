/*
 * Copyright (C) 2019,2020, Xilinx Inc - All rights reserved
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
// ==============================================================
// CTRL
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - enable ap_done interrupt (Read/Write)
//        bit 1  - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - ap_done (COR/TOW)
//        bit 1  - ap_ready (COR/TOW)
//        others - reserved
// 0x20 : Data signal of num_outs
//        bit 31~0 - num_outs[31:0] (Read/Write)
// 0x24 : reserved
// 0x30 : Data signal of start_addr
//        bit 31~0 - start_addr[31:0] (Read/Write)
// 0x34 : Data signal of start_addr
//        bit 31~0 - start_addr[63:32] (Read/Write)
// 0x38 : reserved
// 0x40 : Data signal of ms_maxi_srcbuf
//        bit 31~0 - ms_maxi_srcbuf[31:0] (Read/Write)
// 0x44 : Data signal of ms_maxi_srcbuf
//        bit 31~0 - ms_maxi_srcbuf[63:32] (Read/Write)
// 0x48 : reserved
// 0x50 : Data signal of ms_maxi_dstbuf
//        bit 31~0 - ms_maxi_dstbuf[31:0] (Read/Write)
// 0x54 : Data signal of ms_maxi_dstbuf
//        bit 31~0 - ms_maxi_dstbuf[63:32] (Read/Write)
// 0x58 : reserved
// 0x60 : Data signal of ms_status
//        bit 31~0 - ms_status[31:0] (Read/Write)
// 0x64 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)
#define XV_MULTI_SCALER_CTRL_ADDR_AP_CTRL             0x00
#define XV_MULTI_SCALER_CTRL_ADDR_GIE                 0x04
#define XV_MULTI_SCALER_CTRL_ADDR_IER                 0x08
#define XV_MULTI_SCALER_CTRL_ADDR_ISR                 0x0c
#define XV_MULTI_SCALER_CTRL_ADDR_NUM_OUTS_DATA       0x20
#define XV_MULTI_SCALER_CTRL_BITS_NUM_OUTS_DATA       32
#define XV_MULTI_SCALER_CTRL_ADDR_START_ADDR_DATA     0x30
#define XV_MULTI_SCALER_CTRL_BITS_START_ADDR_DATA     64

#define XV_MULTI_SCALER_CTRL_REGMAP_SIZE              0x038
