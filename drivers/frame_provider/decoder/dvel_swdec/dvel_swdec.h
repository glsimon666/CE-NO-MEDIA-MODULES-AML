/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Dolby Vision FEL Enhancement Layer Software Decoder
 *
 * Minimal HEVC EL residual decoder for S5 single-core HEVC platform.
 * Decodes DV FEL residual (1080p) in software on ARM.
 */

#ifndef _DVEL_SWDEC_H
#define _DVEL_SWDEC_H

#include <linux/types.h>

/* --- HEVC constants --- */

#define DVEL_MAX_WIDTH  1920
#define DVEL_MAX_HEIGHT 1088
#define DVEL_MAX_CTB_SIZE 64
#define DVEL_MAX_CU_SIZE  64
#define DVEL_MAX_TU_SIZE  32
#define DVEL_MAX_REF_PICS 16

/* NAL unit types */
enum dvel_nal_type {
	DVEL_NAL_TRAIL_N   = 0,
	DVEL_NAL_TRAIL_R   = 1,
	DVEL_NAL_VPS       = 32,
	DVEL_NAL_SPS       = 33,
	DVEL_NAL_PPS       = 34,
	DVEL_NAL_EOS       = 36,
	DVEL_NAL_SEI_PREFIX = 39,
	DVEL_NAL_SEI_SUFFIX = 40,
};

/* Slice types */
enum dvel_slice_type {
	DVEL_SLICE_B = 0,
	DVEL_SLICE_P = 1,
	DVEL_SLICE_I = 2,
};

/* Scan order */
enum dvel_scan_idx {
	DVEL_SCAN_DIAG  = 0,
	DVEL_SCAN_HORIZ = 1,
	DVEL_SCAN_VERT  = 2,
};

/* Chroma formats */
enum dvel_chroma_format {
	DVEL_CHROMA_MONO   = 0,
	DVEL_CHROMA_420    = 1,
	DVEL_CHROMA_422    = 2,
	DVEL_CHROMA_444    = 3,
};

/* --- HEVC parameter sets --- */

struct dvel_sps {
	u8 vps_id;
	u8 sps_id;
	u32 pic_width_in_luma_samples;
	u32 pic_height_in_luma_samples;
	u8 chroma_format_idc;
	u8 bit_depth;
	u8 log2_ctb_size;
	u8 log2_min_cb_size;
	u8 log2_diff_max_min_cb;
	u8 log2_min_tb_size;
	u8 log2_diff_max_min_tb;
	u8 max_tb_depth;
	u8 log2_max_trafo_size;
	u8 log2_max_poc_lsb;
	u8 num_short_term_ref_pic_sets;
	bool separate_colour_plane_flag;
	bool sample_adaptive_offset_enabled;
	bool sps_temporal_mvp_enabled_flag;
};

struct dvel_pps {
	u8 pps_id;
	u8 sps_id;
	bool dependent_slice_segments;
	bool cu_qp_delta_enabled;
	s8 init_qp;
	u8 diff_cu_qp_delta_depth;
	s8 cb_qp_offset;
	s8 cr_qp_offset;
	u8 num_extra_slice_header_bits;
	bool output_flag_present;
	bool chroma_qp_offsets_present;
	bool deblocking_filter_control_present;
	bool loop_filter_across_slices_enabled;
	bool tiles_enabled;
	bool entropy_coding_sync_enabled;
};

/* --- CABAC state --- */

/* Number of CABAC context models (HEVC base spec Table 9-39) */
#define DVEL_CABAC_CTX_COUNT 179

struct dvel_cabac {
	u32 low;
	u32 range;
	const u8 *data;
	int data_len;
	int byte_pos;
	int bits_needed;
	u8 state[DVEL_CABAC_CTX_COUNT];  /* (mps<<7) | pStateIdx */
};

/* --- Decoded picture / output --- */

struct dvel_pic {
	s16 *y;
	s16 *u;
	s16 *v;
	int width;
	int height;
	int stride;
	int poc;
	u8 bit_depth;
};

/* --- Main decoder context --- */

struct dvel_ctx {
	/* SPS/PPS store */
	struct dvel_sps sps;
	struct dvel_pps pps;
	bool sps_valid;
	bool pps_valid;

	/* CABAC */
	struct dvel_cabac cabac;

	/* Current decode state */
	int poc;
	int width;
	int height;
	int bit_depth;
	u8 nal_unit_type;

	/* QP */
	int slice_qp;
	int qp_y;

	/* Output residual frame */
	struct dvel_pic *pic;

	/* Decoder statistics */
	u32 frame_count;
	u32 error_count;
};

/* --- API --- */

/* Context-based API (for multi-instance use) */
int dvel_init(struct dvel_ctx *ctx, int width, int height, int bit_depth);
void dvel_exit(struct dvel_ctx *ctx);
int dvel_decode(struct dvel_ctx *ctx, const u8 *data, int size, int poc);

/* Global singleton API (for single-instance use from vh265_fb.c)
 *
 * Integration with vh265_fb.c:
 *   When bypass_dvenl=1 and a NAL with nal_type == 0xA0 (DVEL_NAL) is
 *   found in the stream buffer, the ucode skips EL decode. vh265_fb.c
 *   should copy the EL NAL data and call:
 *     dvel_global_decode(el_nal_data, el_nal_size, poc);
 *
 *   The decoded residual frame becomes available as vframe_s via the
 *   "dveldec" VFM provider. dovi.ko (receiver "dvel") picks it up.
 *
 *   dvel_global_init() is called once during DV FEL stream start
 *   (e.g., in vh265_fb.c's start or first SPS trigger).
 *
 *   dvel_global_exit() is called during stream stop/reset.
 */
int dvel_global_init(int width, int height, int bit_depth);
int dvel_global_decode(const u8 *nal, int size, int poc);
void dvel_global_exit(void);

#endif /* _DVEL_SWDEC_H */
