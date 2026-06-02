/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Dolby Vision FEL Enhancement Layer Software Decoder
 *
 * Minimal HEVC EL residual decoder for kernel integration.
 * Decodes DV FEL 1080p enhancement layer in software.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include "dvel_swdec.h"

static DEFINE_SPINLOCK(dvel_lock);

/* ================================================================
 * Bitstream reader (simple MSB-first get_bits)
 * ================================================================ */

struct bit_reader {
	const u8 *buf;
	int size;
	int byte_pos;
	int bits_left;
	u32 cache;
};

static void br_init(struct bit_reader *br, const u8 *data, int size)
{
	br->buf = data;
	br->size = size;
	br->byte_pos = 0;
	br->bits_left = 0;
	br->cache = 0;
}

static int br_fill(struct bit_reader *br, int n)
{
	while (br->bits_left < n) {
		if (br->byte_pos >= br->size)
			return -1;
		br->cache = (br->cache << 8) | br->buf[br->byte_pos++];
		br->bits_left += 8;
	}
	return 0;
}

static int br_get_bits(struct bit_reader *br, int n)
{
	int ret;

	if (br_fill(br, n) < 0)
		return 0;
	br->bits_left -= n;
	ret = (br->cache >> br->bits_left) & ((1 << n) - 1);
	return ret;
}

static int br_get_bits1(struct bit_reader *br)
{
	return br_get_bits(br, 1);
}

static int br_ue(struct bit_reader *br)
{
	int leading = 0, code;

	while (br_get_bits1(br) == 0 && leading < 32)
		leading++;
	if (leading >= 31)
		return -1;
	code = br_get_bits(br, leading);
	return (1 << leading) - 1 + code;
}

static int br_se(struct bit_reader *br)
{
	int val = br_ue(br);

	if (val < 0)
		return val;
	return (val & 1) ? ((val + 1) >> 1) : -(val >> 1);
}

static int br_byte_aligned(struct bit_reader *br)
{
	return (br->bits_left & 7) == 0;
}

static void br_skip_bits(struct bit_reader *br, int n)
{
	while (n > 0) {
		int skip = min(n, br->bits_left);

		br->bits_left -= skip;
		n -= skip;
		if (n > 0 && br->byte_pos < br->size) {
			br->cache = br->buf[br->byte_pos++];
			br->bits_left = 8;
			n--;
		}
	}
}

static int br_bit_pos(struct bit_reader *br)
{
	return br->byte_pos * 8 - br->bits_left;
}

/* ================================================================
 * NAL parsing helpers
 * ================================================================ */

static const u8 *find_start_code(const u8 *data, int size, int *nal_size)
{
	int i;

	for (i = 2; i < size; i++) {
		if (data[i - 2] == 0 && data[i - 1] == 0 &&
		    (data[i] == 0x01 || data[i] == 0x03)) {
			if (data[i] == 0x01) {
				*nal_size = i - 2;
				return data + i + 1;
			}
		}
	}
	*nal_size = size;
	return NULL;
}

static int parse_nal_header(const u8 *nal, int size,
			    u8 *nal_type, u8 *layer_id)
{
	if (size < 2)
		return -1;
	*nal_type = (nal[0] >> 1) & 0x3f;
	*layer_id = ((nal[0] & 0x01) << 5) | (nal[1] >> 3);
	return 2;
}

static void remove_emul_3bytes(const u8 *src, int src_size,
			       u8 *dst, int *dst_size)
{
	int i, j;

	for (i = 0, j = 0; i < src_size; i++, j++) {
		if (i + 2 < src_size && src[i] == 0 && src[i + 1] == 0 &&
		    src[i + 2] == 0x03) {
			dst[j] = 0;
			dst[j + 1] = 0;
			j++;
			i += 2; /* skip 0x03 */
		} else {
			dst[j] = src[i];
		}
	}
	*dst_size = j;
}

/* ================================================================
 * SPS parsing
 * ================================================================ */

static int parse_sps(struct dvel_ctx *ctx, const u8 *nal, int size)
{
	struct dvel_sps *sps = &ctx->sps;
	struct bit_reader br;
	u8 tmp[4096];
	int tmp_size;
	int i, chroma_sep;
	int max_sub_layers;

	if (size > (int)sizeof(tmp))
		return -1;

	remove_emul_3bytes(nal, size, tmp, &tmp_size);
	br_init(&br, tmp, tmp_size);

	br_get_bits(&br, 4);  /* sps_video_parameter_set_id */
	max_sub_layers = br_get_bits(&br, 3);  /* sps_max_sub_layers_minus1 */
	br_get_bits1(&br);  /* sps_temporal_id_nesting_flag */

	/* profile_tier_level */
	br_get_bits(&br, 2);  /* general_profile_space */
	br_get_bits1(&br);    /* general_tier_flag */
	br_get_bits(&br, 5);  /* general_profile_idc */
	br_get_bits(&br, 32);  /* general_profile_compatibility_flags */
	br_get_bits1(&br);    /* general_progressive_source_flag */
	br_get_bits1(&br);    /* general_interlaced_source_flag */
	br_get_bits1(&br);    /* general_non_packed_constraint_flag */
	br_get_bits1(&br);    /* general_frame_only_constraint_flag */
	br_get_bits(&br, 44); /* general_reserved_zero_44bits + general_reserved_zero_one_bit */
	br_get_bits(&br, 8);  /* general_level_idc */
	for (i = 0; i < max_sub_layers; i++) {
		br_get_bits1(&br);  /* sub_layer_profile_present_flag */
		br_get_bits1(&br);  /* sub_layer_level_present_flag */
	}
	if (max_sub_layers)
		br_get_bits(&br, (8 - max_sub_layers) & 7);  /* reserved_zero bits alignment */
	for (i = 0; i < max_sub_layers; i++) {
		br_get_bits(&br, 2);  /* sub_layer_profile_space */
		br_get_bits1(&br);    /* sub_layer_tier_flag */
		br_get_bits(&br, 5);  /* sub_layer_profile_idc */
		br_get_bits(&br, 32); /* sub_layer_profile_compatibility_flags */
		br_get_bits(&br, 48); /* sub_layer_constraint_flags */
		br_get_bits(&br, 8);  /* sub_layer_level_idc */
	}

	sps->sps_id = br_ue(&br);
	if (sps->sps_id > 15)
		return -1;

	chroma_sep = br_ue(&br);  /* chroma_format_idc */
	sps->separate_colour_plane_flag = 0;
	if (chroma_sep == 3)
		sps->separate_colour_plane_flag = br_get_bits1(&br);
	sps->chroma_format_idc = chroma_sep;

	sps->pic_width_in_luma_samples = br_ue(&br);
	sps->pic_height_in_luma_samples = br_ue(&br);

	if (br_get_bits1(&br)) {  /* conformance_window_flag */
		br_ue(&br);  /* left offset */
		br_ue(&br);  /* right offset */
		br_ue(&br);  /* top offset */
		br_ue(&br);  /* bottom offset */
	}

	sps->bit_depth = br_ue(&br) + 8;  /* bit_depth_luma_minus8 */
	br_ue(&br);  /* bit_depth_chroma_minus8 */
	sps->log2_max_poc_lsb = br_ue(&br) + 4;  /* log2_max_pic_order_cnt_lsb_minus4 */

	sps->log2_min_cb_size = br_ue(&br) + 3;  /* log2_min_luma_coding_block_size_minus3 */
	sps->log2_diff_max_min_cb = br_ue(&br);  /* log2_diff_max_min_luma_coding_block_size */
	sps->log2_ctb_size = sps->log2_min_cb_size + sps->log2_diff_max_min_cb;
	sps->log2_min_tb_size = br_ue(&br) + 2;  /* log2_min_luma_transform_block_size_minus2 */
	sps->log2_diff_max_min_tb = br_ue(&br);  /* log2_diff_max_min_luma_transform_block_size */
	sps->max_tb_depth = br_ue(&br);  /* max_transform_hierarchy_depth */
	sps->log2_max_trafo_size = sps->log2_min_tb_size + sps->log2_diff_max_min_tb;
	if (sps->log2_max_trafo_size > 5)
		sps->log2_max_trafo_size = 5;

	/* Scaling list - properly skip if present */
	if (br_get_bits1(&br)) {  /* scaling_list_enabled_flag */
		if (br_get_bits1(&br)) {  /* sps_scaling_list_data_present_flag */
			int size_id, matrix_id;
			for (size_id = 0; size_id < 4; size_id++) {
				int max_matrix = (size_id == 3) ? 2 : 6;
				for (matrix_id = 0; matrix_id < max_matrix; matrix_id++) {
					int pred = br_get_bits1(&br);  /* scaling_list_pred_mode_flag */
					if (!pred) {
						br_ue(&br);  /* scaling_list_pred_matrix_id_delta */
					} else {
						int coef_num = min(64, 1 << (4 + (size_id << 1)));
						int j;
						for (j = 0; j < coef_num; j++)
							br_se(&br);  /* scaling_list_delta_coef */
					}
				}
			}
		}
	}

	br_get_bits1(&br);  /* amp_enabled_flag */
	sps->sample_adaptive_offset_enabled = br_get_bits1(&br);

	if (br_get_bits1(&br)) {  /* pcm_enabled_flag */
		br_get_bits(&br, 4);  /* pcm_sample_bit_depth_luma_minus1 */
		br_get_bits(&br, 4);  /* pcm_sample_bit_depth_chroma_minus1 */
		br_ue(&br);           /* log2_min_pcm_luma_coding_block_size_minus3 */
		br_ue(&br);           /* log2_diff_max_min_pcm_luma_coding_block_size */
		br_get_bits1(&br);    /* pcm_loop_filter_disabled_flag */
	}

	/* Skip short_term_ref_pic_set(s) - we don't need them for EL decode */
	sps->num_short_term_ref_pic_sets = br_ue(&br);
	{
		int num_rps = sps->num_short_term_ref_pic_sets;
		i = 0;
		while (i < num_rps) {
			if (i != 0 && br_get_bits1(&br)) {
				br_get_bits1(&br);  /* delta_rps_sign */
				br_ue(&br);         /* abs_delta_rps_minus1 */
			} else {
				int num_neg = br_ue(&br);
				int num_pos = br_ue(&br);
				int j;
				for (j = 0; j < num_neg; j++) {
					br_ue(&br);  /* delta_poc_s0_minus1 */
					br_get_bits1(&br);  /* used_by_curr_pic_s0_flag */
				}
				for (j = 0; j < num_pos; j++) {
					br_ue(&br);  /* delta_poc_s1_minus1 */
					br_get_bits1(&br);  /* used_by_curr_pic_s1_flag */
				}
			}
			i++;
		}
	}

	br_get_bits1(&br);  /* long_term_ref_pics_present_flag */
	/* Skip LT ref pics */
	ctx->sps.sps_temporal_mvp_enabled_flag = br_get_bits1(&br);  /* sps_temporal_mvp_enabled_flag */
	br_get_bits1(&br);  /* strong_intra_smoothing_enabled_flag */

	/* VUI parameters (skip) */
	if (br_get_bits1(&br)) {
		/* vui_parameters_present_flag */
		/* skip - not needed for decode */
	}

	/* Extension flag */
	if (br_get_bits1(&br)) {
		/* sps_extension_flag */
		/* skip extension */
	}

	sps->log2_max_trafo_size = min_t(u8, sps->log2_max_trafo_size, 5);
	ctx->width = sps->pic_width_in_luma_samples;
	ctx->height = sps->pic_height_in_luma_samples;
	ctx->bit_depth = sps->bit_depth;
	ctx->sps_valid = true;

	return 0;
}

/* ================================================================
 * PPS parsing
 * ================================================================ */

static int parse_pps(struct dvel_ctx *ctx, const u8 *nal, int size)
{
	struct dvel_pps *pps = &ctx->pps;
	struct bit_reader br;
	u8 tmp[4096];
	int tmp_size;

	if (size > (int)sizeof(tmp))
		return -1;

	remove_emul_3bytes(nal, size, tmp, &tmp_size);
	br_init(&br, tmp, tmp_size);

	pps->pps_id = br_ue(&br);
	pps->sps_id = br_ue(&br);

	pps->dependent_slice_segments = br_get_bits1(&br);
	pps->output_flag_present = br_get_bits1(&br);
	pps->num_extra_slice_header_bits = br_get_bits(&br, 3);

	br_get_bits1(&br);  /* sign_data_hiding_enabled_flag (skip) */

	pps->init_qp = br_se(&br);  /* init_qp_minus26 */
	br_get_bits1(&br);  /* constrained_intra_pred_flag (skip) */
	br_get_bits1(&br);  /* transform_skip_enabled_flag (skip) */

	pps->cu_qp_delta_enabled = br_get_bits1(&br);
	if (pps->cu_qp_delta_enabled)
		pps->diff_cu_qp_delta_depth = br_ue(&br);

	pps->cb_qp_offset = br_se(&br);  /* pps_cb_qp_offset */
	pps->cr_qp_offset = br_se(&br);  /* pps_cr_qp_offset */
	pps->chroma_qp_offsets_present = br_get_bits1(&br);

	/* weighted_pred_flag / weighted_bipred_flag / transquant_bypass_enabled */
	br_get_bits(&br, 3);  /* skip */

	/* tiles_enabled_flag */
	pps->tiles_enabled = br_get_bits1(&br);
	pps->entropy_coding_sync_enabled = br_get_bits1(&br);

	pps->loop_filter_across_slices_enabled = br_get_bits1(&br);
	pps->deblocking_filter_control_present = false;
	if (pps->loop_filter_across_slices_enabled)
		pps->deblocking_filter_control_present = br_get_bits1(&br);
	if (pps->deblocking_filter_control_present) {
		if (br_get_bits1(&br)) {  /* pps_deblocking_filter_disabled_flag */
			br_se(&br);  /* pps_beta_offset_div2 */
			br_se(&br);  /* pps_tc_offset_div2 */
		}
	}
	/* Skip remainder of PPS */
	ctx->pps_valid = true;

	return 0;
}

/* ================================================================
 * CABAC init tables (from FFmpeg hevc/cabac.c)
 * ================================================================ */

static const u8 dvel_cabac_init[3][DVEL_CABAC_CTX_COUNT] = {
#include "dvel_cabac_init.h"
};

/* State transition tables (HEVC spec 9.3.4.2.1) */
static const u8 trans_idx_mps[64] = {
	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
	17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
	49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 62, 63,
};

static const u8 trans_idx_lps[64] = {
	 0,  0,  1,  2,  2,  4,  4,  5,  6,  7,  8,  9,  9, 11, 11, 12,
	13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
	24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
	33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
};

/* rangeTabLPS[qRangeIdx][pStateIdx] (HEVC spec Table 9-38) */
static const u8 range_tab_lps[4][64] = {
#include "dvel_range_tab.h"
};

/* ================================================================
 * CABAC implementation (matching FFmpeg CABAC_BITS=16 model)
 * ================================================================ */

#define DVEL_CABAC_BITS 16
#define DVEL_CABAC_MASK (~((1U << (DVEL_CABAC_BITS + 1)) - 1)) /* 0xFFFE0000 */

/* Initialize CABAC context states from init_value and slice_qp
 * Follows HEVC spec 9.3.4.2.3 / FFmpeg cabac_init_state()
 */
static void cabac_init_state(struct dvel_cabac *c, int init_type, int slice_qp)
{
	int i;

	for (i = 0; i < DVEL_CABAC_CTX_COUNT; i++) {
		int init_value = dvel_cabac_init[init_type][i];
		int m = ((init_value >> 4) & 0xf) * 5 - 45;
		int n = ((init_value & 0xf) << 3) - 16;
		int pre;

		/* Clip qp to valid range */
		if (slice_qp < 0) slice_qp = 0;
		if (slice_qp > 51) slice_qp = 51;

		pre = 2 * (((m * slice_qp) >> 4) + n) - 127;

		/* Handle negative -> positive */
		if (pre < 0)
			pre = -pre;

		/* Clip to [0, 124], preserving oddness */
		if (pre > 124)
			pre = 124 + (pre & 1);

		/* Convert to standard state encoding: (mps<<7) | pStateIdx
		 * pre ranges 0..125 where:
		 *   pre < 64  → mps=0, pStateIdx = pre
		 *   pre >= 64 → mps=1, pStateIdx = pre - 64
		 */
		if (pre >= 64)
			c->state[i] = (1 << 7) | (pre - 64);
		else
			c->state[i] = pre;
	}
}

/* Read one byte and insert into low during refill.
 * Matches FFmpeg's refill() with CABAC_BITS=16.
 */
static void cabac_refill(struct dvel_cabac *c)
{
	if (c->byte_pos >= c->data_len)
		return;
	/* Add 16 bits: byte0<<9 + byte1<<1 */
	c->low += c->data[c->byte_pos++] << 9;
	if (c->byte_pos < c->data_len)
		c->low += c->data[c->byte_pos++] << 1;
	c->low -= DVEL_CABAC_MASK;
}

/* Initialize CABAC decoder.
 * Matches FFmpeg ff_init_cabac_decoder() with CABAC_BITS=16.
 */
static void cabac_init(struct dvel_cabac *c,
		       const u8 *data, int len,
		       int init_type, int slice_qp)
{
	c->data = data;
	c->data_len = len;
	c->byte_pos = 0;
	c->bits_needed = 0;
	c->range = 0x1FE;

	/* Compute CABAC states from init table + slice_qp */
	cabac_init_state(c, init_type, slice_qp);

	/* Preload low: matches FFmpeg init:
	 * low = byte0<<18 + byte1<<10 + (1<<9 or byte2<<2 + 2)
	 * We align to 2-byte boundary for simplicity.
	 */
	c->low = 0;
	if (c->byte_pos >= c->data_len)
		return;
	c->low = c->data[c->byte_pos++] << 18;
	if (c->byte_pos >= c->data_len)
		return;
	c->low += c->data[c->byte_pos++] << 10;
	/* Add 1<<9 offset + 2 (matching FFmpeg CABAC_BITS=16 init) */
	c->low += (1 << 9) + 2;
}

/* Decode one CABAC bin.
 * Uses standard HEVC state encoding: (mps<<7) | pStateIdx.
 * Arithmetic matches FFmpeg get_cabac_inline() with CABAC_BITS=16.
 */
static int cabac_decode_bin(struct dvel_cabac *c, int ctx_idx)
{
	u8 state_byte = c->state[ctx_idx];
	int mps = state_byte >> 7;
	int pstate = state_byte & 0x3f;
	int range_lps = range_tab_lps[(c->range >> 6) & 3][pstate];
	int lps_mask;

	c->range -= range_lps;

	/* Compare: (range<<17) vs low */
	lps_mask = ((c->range << (DVEL_CABAC_BITS + 1)) - c->low) >> 31;

	c->low -= (c->range << (DVEL_CABAC_BITS + 1)) & lps_mask;
	c->range += (range_lps - c->range) & lps_mask;

	/* State transition */
	if (lps_mask) {
		pstate = trans_idx_lps[pstate];
		if (pstate == 0)
			mps = !mps;
	} else {
		pstate = trans_idx_mps[pstate];
	}
	c->state[ctx_idx] = (mps << 7) | pstate;

	/* Renormalize (use simple shift, not norm_shift table) */
	while (c->range < 0x100) {
		c->range <<= 1;
		c->low <<= 1;
		if (!(c->low & DVEL_CABAC_MASK) && c->byte_pos < c->data_len) {
			/* Refill when top (CABAC_BITS+1) bits are zero */
			if (c->byte_pos + 1 < c->data_len) {
				c->low += c->data[c->byte_pos++] << 9;
				c->low += c->data[c->byte_pos++] << 1;
			} else {
				c->low += c->data[c->byte_pos++] << 9;
			}
			c->low -= DVEL_CABAC_MASK;
		}
	}

	return (mps ^ ((unsigned int)lps_mask >> 31)) & 1;
}

/* Decode bypass bin (no probability model, 50/50 split).
 * Matches FFmpeg get_cabac_bypass().
 */
static int cabac_decode_bypass(struct dvel_cabac *c)
{
	int range;

	c->low += c->low;
	if (!(c->low & DVEL_CABAC_MASK) && c->byte_pos < c->data_len)
		cabac_refill(c);

	range = c->range << (DVEL_CABAC_BITS + 1);
	if (c->low < range)
		return 0;
	c->low -= range;
	return 1;
}

/* Decode terminate bin.
 * Matches FFmpeg get_cabac_terminate().
 */
static int cabac_decode_terminate(struct dvel_cabac *c)
{
	c->range -= 2;
	if (c->low < c->range << (DVEL_CABAC_BITS + 1)) {
		/* Not terminated - renormalize */
		while (c->range < 0x100) {
			c->range <<= 1;
			c->low <<= 1;
			if (!(c->low & DVEL_CABAC_MASK) && c->byte_pos < c->data_len) {
				if (c->byte_pos + 1 < c->data_len) {
					c->low += c->data[c->byte_pos++] << 9;
					c->low += c->data[c->byte_pos++] << 1;
				} else {
					c->low += c->data[c->byte_pos++] << 9;
				}
				c->low -= DVEL_CABAC_MASK;
			}
		}
		return 0;
	}
	/* Terminated */
	return 1;
}

#undef DVEL_CABAC_BITS
#undef DVEL_CABAC_MASK

/* ================================================================
 * Slice header parsing (on already-emul-removed RBSP data)
 * ================================================================ */

/* Skip a short_term_ref_pic_set from the bitstream.
 * st_rps_idx: index of this set (0-based). If !=0, prediction flag is present.
 * Returns 0 on success.
 */
static int skip_short_term_ref_pic_set(struct bit_reader *br, int st_rps_idx)
{
	if (st_rps_idx != 0 && br_get_bits1(br)) {  /* inter_ref_pic_set_prediction_flag */
		br_get_bits1(br);  /* delta_rps_sign */
		br_ue(br);         /* abs_delta_rps_minus1 */
		/* skip used_by_curr_pic/use_delta flags - we don't track NumDeltaPocs */
	} else {
		int num_neg = br_ue(br);
		int num_pos = br_ue(br);
		int j;
		for (j = 0; j < num_neg; j++) {
			br_ue(br);  /* delta_poc_s0_minus1 */
			br_get_bits1(br);  /* used_by_curr_pic_s0_flag */
		}
		for (j = 0; j < num_pos; j++) {
			br_ue(br);  /* delta_poc_s1_minus1 */
			br_get_bits1(br);  /* used_by_curr_pic_s1_flag */
		}
	}
	return 0;
}

/* Parse slice header from RBSP (emul already removed).
 * Returns bit position where slice header ends (CABAC data start), or < 0 on error.
 * Output slice_type through out_slice_type.
 * Context fields updated: slice_qp, qp_y.
 * Fields consumed in HEVC spec order (7.3.6.1).
 */
static int parse_slice_header_rbsp(struct dvel_ctx *ctx,
				   const u8 *rbsp, int rbsp_size,
				   int *out_slice_type)
{
	struct bit_reader br;
	int slice_type;
	int first_slice, dep_slice = 0;
	int i;

	br_init(&br, rbsp, rbsp_size);

	first_slice = br_get_bits1(&br);  /* first_slice_segment_in_pic_flag */

	/* skip no_output_of_prior_pics_flag - not needed for TRAIL_N/R (non-IDR) */

	br_ue(&br);  /* slice_pic_parameter_set_id */

	if (!first_slice) {
		dep_slice = br_get_bits1(&br);  /* dependent_slice_segment_flag */
		/* slice_segment_address: u(v) with Ceil(Log2(CtbSize/MinCbSize)) bits */
		{
			int addr_bits = ctx->sps.log2_ctb_size - ctx->sps.log2_min_cb_size;
			if (addr_bits > 0)
				br_get_bits(&br, addr_bits);
		}
	}

	if (!dep_slice) {
		/* num_extra_slice_header_bits from PPS */
		for (i = 0; i < ctx->pps.num_extra_slice_header_bits; i++)
			br_get_bits1(&br);

		slice_type = br_ue(&br);

		if (ctx->pps.output_flag_present)
			br_get_bits1(&br);  /* pic_output_flag */

		if (ctx->sps.separate_colour_plane_flag)
			br_get_bits(&br, 2);  /* colour_plane_id */

		/* slice_pic_order_cnt_lsb - always present for non-IDR (TRAIL_N/R) */
		if (ctx->sps.log2_max_poc_lsb > 0)
			br_get_bits(&br, ctx->sps.log2_max_poc_lsb);

		/* short_term_ref_pic_set */
		{
			int st_rps_sps_flag = br_get_bits1(&br);
			if (!st_rps_sps_flag) {
				skip_short_term_ref_pic_set(&br,
					ctx->sps.num_short_term_ref_pic_sets);
			} else {
				br_ue(&br);  /* short_term_ref_pic_set_idx */
			}
		}

		/* long_term_ref_pics */
		if (br_get_bits1(&br)) {  /* long_term_ref_pics_present_flag */
			int num_lt_sps = br_ue(&br);
			int num_lt_pics = br_ue(&br);
			for (i = 0; i < num_lt_sps + num_lt_pics; i++) {
				if (i < num_lt_sps)
					br_get_bits(&br, ctx->sps.log2_max_poc_lsb);
				else
					br_get_bits(&br, ctx->sps.log2_max_poc_lsb);
				br_get_bits1(&br);  /* used_by_curr_pic_lt_flag */
				if (br_get_bits1(&br))  /* delta_poc_msb_present_flag */
					br_ue(&br);  /* delta_poc_msb_cycle_lt */
			}
		}

		/* slice_temporal_mvp_enabled_flag */
		if (ctx->sps.sps_temporal_mvp_enabled_flag)
			br_get_bits1(&br);

		/* SAO flags (before QP delta in HEVC spec order) */
		if (ctx->sps.sample_adaptive_offset_enabled) {
			/* slice_sao_luma_flag (just read, not used) */
			br_get_bits1(&br);
			if (ctx->sps.chroma_format_idc != DVEL_CHROMA_MONO)
				br_get_bits1(&br);  /* slice_sao_chroma_flag */
		}

		/* QP delta */
		ctx->slice_qp = br_se(&br) + ctx->pps.init_qp + 26;
		ctx->qp_y = ctx->slice_qp;

		/* Chroma QP offset */
		if (ctx->pps.chroma_qp_offsets_present) {
			br_se(&br);  /* slice_cb_qp_offset */
			br_se(&br);  /* slice_cr_qp_offset */
		}

		/* Deblocking filter */
		if (ctx->pps.deblocking_filter_control_present) {
			if (br_get_bits1(&br)) {  /* deblocking_filter_override_flag */
				br_se(&br);  /* deblocking_filter_offset */
				br_se(&br);  /* deblocking_filter_beta_offset */
			}
		}

		/* Loop filter across slices */
		if (ctx->pps.loop_filter_across_slices_enabled)
			br_get_bits1(&br);  /* slice_loop_filter_across_slices_enabled_flag */
	}

	/* Entry point offsets (if tiles or entropy sync) - part of slice_segment_data */
	if (ctx->pps.tiles_enabled || ctx->pps.entropy_coding_sync_enabled) {
		int num_entry = br_ue(&br);  /* num_entry_point_offsets */
		if (num_entry > 0) {
			int offset_len = br_ue(&br) + 1;  /* offset_len_minus1 */
			for (i = 0; i < num_entry; i++)
				br_get_bits(&br, offset_len);  /* entry_point_offset_minus1[i] */
		}
	}

	if (out_slice_type)
		*out_slice_type = slice_type;

	/* Return bit position. Caller rounds up to byte for CABAC offset
	 * (byte_alignment in header ensures proper alignment).
	 */
	return br_bit_pos(&br);
}

/* ================================================================
 * Residual decoding
 * ================================================================ */

/* Scan tables */
static const u8 scan_diag_4x4_x[16] = { 0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 1, 2, 3, 2, 3, 3 };
static const u8 scan_diag_4x4_y[16] = { 0, 1, 0, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 3, 2, 3 };
static const u8 scan_horiz_4x4_x[16] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 };
static const u8 scan_horiz_4x4_y[16] = { 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 };
static const u8 scan_vert_4x4_x[16] = { 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 };
static const u8 scan_vert_4x4_y[16] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 };

static const u8 scan_1x1[1] = { 0 };
static const u8 diag_scan2x2_x[4] = { 0, 0, 1, 1 };
static const u8 diag_scan2x2_y[4] = { 0, 1, 0, 1 };
static const u8 diag_scan2x2_inv[2][2] = { {0, 1}, {2, 3} };
static const u8 diag_scan4x4_inv[4][4] = {
	{0,  1,  4,  8},
	{2,  3,  5,  9},
	{6,  7,  10, 12},
	{11, 13, 14, 15},
};
static const u8 diag_scan8x8_inv[8][8] = {
	{0,  1,  8,  16, 27, 37, 46, 54},
	{2,  3,  9,  17, 28, 38, 47, 55},
	{4,  5,  10, 18, 29, 39, 48, 56},
	{6,  7,  11, 19, 30, 40, 49, 57},
	{12, 13, 14, 20, 31, 41, 50, 58},
	{21, 22, 23, 24, 32, 42, 51, 59},
	{33, 34, 35, 36, 43, 52, 60, 61},
	{44, 45, 53, 62, 63, 15, 25, 26},
};

static const u8 horiz_scan2x2_x[4] = { 0, 1, 0, 1 };
static const u8 horiz_scan2x2_y[4] = { 0, 0, 1, 1 };
static const u8 horiz_scan4x4_x[16] = {
	0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3
};
static const u8 horiz_scan4x4_y[16] = {
	0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3
};
static const u8 horiz_scan8x8_inv[8][8] = {
	{0,  1,  2,  3,  4,  5,  6,  7},
	{8,  9,  10, 11, 12, 13, 14, 15},
	{16, 17, 18, 19, 20, 21, 22, 23},
	{24, 25, 26, 27, 28, 29, 30, 31},
	{32, 33, 34, 35, 36, 37, 38, 39},
	{40, 41, 42, 43, 44, 45, 46, 47},
	{48, 49, 50, 51, 52, 53, 54, 55},
	{56, 57, 58, 59, 60, 61, 62, 63},
};

static const u8 ff_hevc_diag_scan4x4_x[16] = {
	0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 1, 2, 3, 2, 3, 3
};
static const u8 ff_hevc_diag_scan4x4_y[16] = {
	0, 1, 0, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 3, 2, 3
};
/* 8x8 diagonal scan (not needed for base EL implementation)
 * Will be populated in full implementation */

/* Dequantization */
static const u8 level_scale[] = { 40, 45, 51, 57, 64, 72 };

/* HEVC spec Table 8-11: QpC = qPiTable[Clip3(0, 57, QP_Y + pps_cb/cr_qp_offset)] */
static int dvel_chroma_qp(int qp_y, int is_cr)
{
	static const u8 qpi_table_cb[58] = {
		 0,  1,  2,  3,  4,  5,  6,  7,
		 8,  9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 29, 30,
		31, 32, 33, 33, 34, 35, 36, 37,
		37, 38, 39, 40, 41, 42, 42, 43,
		44, 45, 46, 46, 47, 48, 49, 50,
		50, 51
	};
	static const u8 qpi_table_cr[58] = {
		 0,  1,  2,  3,  4,  5,  6,  7,
		 8,  9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31,
		32, 33, 34, 35, 36, 37, 37, 38,
		39, 40, 41, 42, 43, 44, 45, 46,
		47, 48, 48, 49, 50, 51, 51, 52,
		53, 54
	};

	if (qp_y < 0)
		qp_y = 0;
	if (qp_y > 57)
		qp_y = 57;
	return is_cr ? qpi_table_cr[qp_y] : qpi_table_cb[qp_y];
}

static const u8 rem6[78] = {
	0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3,
	4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1,
	2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5,
	0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3,
	4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2
};
static const u8 div6[78] = {
	0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
	2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5,
	5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10,
	10, 10, 11, 11, 11, 11, 11, 11, 12, 12
};

/* Context index map for significant_coeff_flag */
static const u8 ctx_idx_map[5 * 16] = {
	0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8,
	1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	2, 1, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2
};

/* CABAC query functions (from ffmpeg/hevc/cabac.c) */
static int significant_coeff_group_flag_decode(struct dvel_cabac *c, int ctx)
{
	return cabac_decode_bin(c, 89 + ctx);
}

static int significant_coeff_flag_decode(struct dvel_cabac *c, int x_c, int y_c,
					  int offset, const u8 *ctx_idx_map_p)
{
	int ctx = ctx_idx_map_p[(y_c << 2) + x_c] + offset;

	return cabac_decode_bin(c, ctx);
}

static int significant_coeff_flag_decode_0(struct dvel_cabac *c, int idx)
{
	return cabac_decode_bin(c, idx);
}

static int coeff_abs_level_greater1_flag_decode(struct dvel_cabac *c, int idx)
{
	return cabac_decode_bin(c, 137 + idx);
}

static int coeff_abs_level_greater2_flag_decode(struct dvel_cabac *c, int idx)
{
	/* Context 161..166, after 24 greater1 contexts (137..160) */
	return cabac_decode_bin(c, 161 + idx);
}

static int coeff_abs_level_remaining_decode(struct dvel_cabac *c, int c_rice_param)
{
	int prefix = 0, suffix;
	int threshold = 3 + 4 * c_rice_param;

	while (cabac_decode_bypass(c)) {
		prefix++;
		if (prefix >= threshold)
			break;
	}
	if (prefix < threshold) {
		suffix = 0;
		if (c_rice_param > 0) {
			int rp = c_rice_param;
			while (rp--)
				suffix = (suffix << 1) | cabac_decode_bypass(c);
		}
		return (prefix >> c_rice_param) + suffix;
	}
	suffix = 0;
	{
		int suffix_len = prefix - 5 - (4 * c_rice_param);
		int j;
		for (j = 0; j < suffix_len; j++)
			suffix = (suffix << 1) | cabac_decode_bypass(c);
	}
	return (1 << c_rice_param) + prefix - threshold + suffix;
}

static int coeff_sign_flag_decode(struct dvel_cabac *c)
{
	return cabac_decode_bypass(c);
}

/* ================================================================
 * Scan position helpers (from FFmpeg hevc/cabac.c)
 * ================================================================ */

/* Helper to get diagonal scan position from inverse table */
#define DVEL_GET_COORD(scan_x, scan_y, cg_x, cg_y, sub_pos) \
	do { \
		x_c = (cg_x << 2) + (scan_x)[sub_pos]; \
		y_c = (cg_y << 2) + (scan_y)[sub_pos]; \
	} while (0)

/* Decode last_significant_coeff_x/y prefix + suffix */
static void last_sig_coeff_decode(struct dvel_cabac *cabac, int log2_trafo_size,
				  int *last_x, int *last_y)
{
	int prefix_x = 0, prefix_y = 0;
	int suffix_x = 0, suffix_y = 0;
	int ctx_last_offset;

	/* last_significant_coeff_x_prefix */
	ctx_last_offset = (log2_trafo_size << 3) + 3;
	prefix_x = 0;
	while (cabac_decode_bin(cabac, 53 + (prefix_x >> 1))) {
		prefix_x++;
		if (prefix_x >= ctx_last_offset)
			break;
	}

	/* last_significant_coeff_y_prefix */
	prefix_y = 0;
	while (cabac_decode_bin(cabac, 71 + (prefix_y >> 1))) {
		prefix_y++;
		if (prefix_y >= ctx_last_offset)
			break;
	}

	/* Suffix decode (if prefix > 3) */
	if (prefix_x > 3) {
		int suffix_len = (prefix_x >> 1) - 1;
		int i;
		for (i = 0; i < suffix_len; i++)
			suffix_x = (suffix_x << 1) | cabac_decode_bypass(cabac);
		*last_x = ((2 + (prefix_x & 1)) << (suffix_len)) + suffix_x;
	} else {
		*last_x = prefix_x;
	}

	if (prefix_y > 3) {
		int suffix_len = (prefix_y >> 1) - 1;
		int i;
		for (i = 0; i < suffix_len; i++)
			suffix_y = (suffix_y << 1) | cabac_decode_bypass(cabac);
		*last_y = ((2 + (prefix_y & 1)) << (suffix_len)) + suffix_y;
	} else {
		*last_y = prefix_y;
	}
}

/* ================================================================
 * Main residual coding (from FFmpeg hevc/cabac.c ff_hevc_hls_residual_coding)
 * ================================================================ */

/* Coefficients context index map for significant_coeff_flag */
static const u8 dvel_ctx_idx_map[80] = {
	/* log2_trafo_size == 2 */
	0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8,
	/* prev_sig == 0 */
	1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	/* prev_sig == 1 */
	2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	/* prev_sig == 2 */
	2, 1, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0, 2, 1, 0, 0,
	/* default (transform_skip) */
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
};

/* 8x8 diagonal scan (CG coordinates) */
static const u8 dvel_diag8_x[64] = {
	0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4, 0,
	1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3,
	4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6,
	7, 3, 4, 5, 6, 7, 4, 5, 6, 7, 5, 6, 7, 6, 7, 7,
};
static const u8 dvel_diag8_y[64] = {
	0, 1, 0, 2, 1, 0, 3, 2, 1, 0, 4, 3, 2, 1, 0, 5,
	4, 3, 2, 1, 0, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4,
	3, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 7, 6, 5, 4, 3,
	2, 7, 6, 5, 4, 3, 7, 6, 5, 4, 7, 6, 5, 7, 6, 7,
};

/* 4x4 diagonal scan (sub-block positions) */
static const u8 dvel_diag4_x[16] = {
	0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 1, 2, 3, 2, 3, 3,
};
static const u8 dvel_diag4_y[16] = {
	0, 1, 0, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 3, 2, 3,
};

/* 2x2 CG diagonal scan (for 8x8 TU) */
static const u8 dvel_cg2_x[4] = { 0, 0, 1, 1 };
static const u8 dvel_cg2_y[4] = { 0, 1, 0, 1 };

/* 4x4 CG diagonal scan (for 16x16 TU) */
/* Same as dvel_diag4 but used as CG-level scan */

/* Inverse diagonal scan maps for position -> scan index */
static const u8 dvel_inv_diag4[4][4] = {
	{0,  1,  4,  8},
	{2,  3,  5,  9},
	{6,  7,  10, 12},
	{11, 13, 14, 15},
};
static const u8 dvel_inv_diag8[8][8] = {
	{0,  1,  8,  16, 27, 37, 46, 54},
	{2,  3,  9,  17, 28, 38, 47, 55},
	{4,  5,  10, 18, 29, 39, 48, 56},
	{6,  7,  11, 19, 30, 40, 49, 57},
	{12, 13, 14, 20, 31, 41, 50, 58},
	{21, 22, 23, 24, 32, 42, 51, 59},
	{33, 34, 35, 36, 43, 52, 60, 61},
	{44, 45, 53, 62, 63, 15, 25, 26},
};
static const u8 dvel_inv_cg2[2][2] = { {0, 1}, {2, 3} };
/* CG scan for 16x16 uses same diagonal as 4x4 sub-block */

/* Horizontal scan tables (for SCAN_HORIZ / SCAN_VERT) */
static const u8 dvel_horiz4_x[16] = {
	0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3,
};
static const u8 dvel_horiz4_y[16] = {
	0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
};
static const u8 dvel_horiz8_inv[8][8] = {
	{0,  1,  2,  3,  4,  5,  6,  7},
	{8,  9,  10, 11, 12, 13, 14, 15},
	{16, 17, 18, 19, 20, 21, 22, 23},
	{24, 25, 26, 27, 28, 29, 30, 31},
	{32, 33, 34, 35, 36, 37, 38, 39},
	{40, 41, 42, 43, 44, 45, 46, 47},
	{48, 49, 50, 51, 52, 53, 54, 55},
	{56, 57, 58, 59, 60, 61, 62, 63},
};
static const u8 dvel_cg_horiz2_x[4] = { 0, 1, 0, 1 };
static const u8 dvel_cg_horiz2_y[4] = { 0, 0, 1, 1 };

/* Full residual coding for one transform unit
 * Decodes all coefficient levels via CABAC, dequantizes, and stores in coeffs[].
 * Simplified for DV FEL EL: no RDPCM, no persistent_rice, no transform_skip.
 */
static void residual_coding(struct dvel_ctx *ctx,
			    int16_t *coeffs, int log2_trafo_size,
			    int c_idx)
{
	struct dvel_cabac *cabac = &ctx->cabac;
	int trafo_size = 1 << log2_trafo_size;
	int last_sig_x, last_sig_y;
	int x_cg_last, y_cg_last;
	int scan_idx = DVEL_SCAN_DIAG;
	const u8 *scan_x_cg, *scan_y_cg;
	const u8 *scan_x_off, *scan_y_off;
	u8 sig_cg_flag[8][8] = {{0}};
	int num_coeff = 0;
	int num_last_subset;
	int qp, scale, shift, add;
	int i;

	memset(coeffs, 0, trafo_size * trafo_size * sizeof(int16_t));

	/* --- Decode last significant coefficient position --- */
	last_sig_coeff_decode(cabac, log2_trafo_size, &last_sig_x, &last_sig_y);

	x_cg_last = last_sig_x >> 2;
	y_cg_last = last_sig_y >> 2;

	/* --- Setup scan tables --- */
	{
		int last_x_c = last_sig_x & 3;
		int last_y_c = last_sig_y & 3;

		if (trafo_size == 4) {
			scan_x_cg = scan_1x1;
			scan_y_cg = scan_1x1;
			scan_x_off = dvel_diag4_x;
			scan_y_off = dvel_diag4_y;
			num_coeff = dvel_inv_diag4[last_y_c][last_x_c];
		} else if (trafo_size == 8) {
			scan_x_cg = dvel_cg2_x;
			scan_y_cg = dvel_cg2_y;
			scan_x_off = dvel_diag4_x;
			scan_y_off = dvel_diag4_y;
			num_coeff = dvel_inv_diag4[last_y_c][last_x_c]
				  + (dvel_inv_cg2[y_cg_last][x_cg_last] << 4);
		} else if (trafo_size == 16) {
			scan_x_cg = dvel_diag4_x;
			scan_y_cg = dvel_diag4_y;
			scan_x_off = dvel_diag4_x;
			scan_y_off = dvel_diag4_y;
			num_coeff = dvel_inv_diag4[last_y_c][last_x_c]
				  + (dvel_inv_diag4[y_cg_last][x_cg_last] << 4);
		} else { /* trafo_size == 32 */
			scan_x_cg = dvel_diag8_x;
			scan_y_cg = dvel_diag8_y;
			scan_x_off = dvel_diag4_x;
			scan_y_off = dvel_diag4_y;
			num_coeff = dvel_inv_diag4[last_y_c][last_x_c]
				  + (dvel_inv_diag8[y_cg_last][x_cg_last] << 4);
		}
	}
	num_coeff++;
	num_last_subset = (num_coeff - 1) >> 4;

	/* --- Process each subblock (16 coeffs) --- */
	for (i = num_last_subset; i >= 0; i--) {
		int n, m;
		int x_cg, y_cg, x_c = 0, y_c = 0;
		int implicit_non_zero = 0;
		int offset = i << 4;
		int n_end;
		u8 sig_flag_idx[16];
		int nb_sig = 0;

		x_cg = scan_x_cg[i];
		y_cg = scan_y_cg[i];

		/* Decode significant_coeff_group_flag */
		if (i < num_last_subset && i > 0) {
			int ctx_cg = 0;
			if (x_cg < (1 << (log2_trafo_size - 2)) - 1)
				ctx_cg += sig_cg_flag[x_cg + 1][y_cg];
			if (y_cg < (1 << (log2_trafo_size - 2)) - 1)
				ctx_cg += sig_cg_flag[x_cg][y_cg + 1];
			sig_cg_flag[x_cg][y_cg] = significant_coeff_group_flag_decode(cabac, ctx_cg);
			implicit_non_zero = 1;
		} else {
			sig_cg_flag[x_cg][y_cg] =
				((x_cg == x_cg_last && y_cg == y_cg_last) ||
				 (x_cg == 0 && y_cg == 0));
		}

		{
			int last_scan_pos = num_coeff - offset - 1;
			if (i == num_last_subset) {
				n_end = last_scan_pos - 1;
				sig_flag_idx[0] = last_scan_pos;
				nb_sig = 1;
			} else {
				n_end = 15;
			}

			/* prev_sig context for sig_coeff_flag */
			int prev_sig = 0;
			if (x_cg < ((1 << log2_trafo_size) - 1) >> 2)
				prev_sig = !!sig_cg_flag[x_cg + 1][y_cg];
			if (y_cg < ((1 << log2_trafo_size) - 1) >> 2)
				prev_sig += (!!sig_cg_flag[x_cg][y_cg + 1] << 1);

			if (sig_cg_flag[x_cg][y_cg] && n_end >= 0) {
				const u8 *ctx_map;
				int scf_off = 0;

				if (log2_trafo_size == 2) {
					ctx_map = &dvel_ctx_idx_map[0];
				} else {
					ctx_map = &dvel_ctx_idx_map[(prev_sig + 1) << 4];
					if (c_idx == 0) {
						if (x_cg > 0 || y_cg > 0)
							scf_off += 3;
						if (log2_trafo_size == 3)
							scf_off += 9;
						else
							scf_off += 21;
					} else {
						if (log2_trafo_size == 3)
							scf_off += 9;
						else
							scf_off += 12;
					}
				}

				for (n = n_end; n > 0; n--) {
					x_c = scan_x_off[n];
					y_c = scan_y_off[n];
					if (significant_coeff_flag_decode(cabac, x_c, y_c,
									   93 + scf_off, ctx_map)) {
						sig_flag_idx[nb_sig] = n;
						nb_sig++;
						implicit_non_zero = 0;
					}
				}

				/* Position 0 (DC) */
				if (implicit_non_zero == 0) {
					int dc_off = (i == 0) ? 0 : (2 + scf_off);
					if (significant_coeff_flag_decode_0(cabac, 93 + dc_off)) {
						sig_flag_idx[nb_sig] = 0;
						nb_sig++;
					}
				} else {
					sig_flag_idx[nb_sig] = 0;
					nb_sig++;
				}
			}
		}

		n_end = nb_sig;

		/* --- Decode coefficient levels --- */
		if (n_end) {
			int ctx_set = (i > 0 && c_idx == 0) ? 2 : 0;
			int greater1_ctx = 1;
			int c_rice = 0;
			int first_g1_idx = -1;
			u8 g1_flag[8] = {0};
			u8 signs[16] = {0};
			int sign_hidden;
			int first_nz_pos;
			int last_nz_pos;

			last_nz_pos = sig_flag_idx[0];

			if (!(i == num_last_subset) && greater1_ctx == 0)
				ctx_set++;
			greater1_ctx = 1;

			/* Decode greater1 flags (max 8) */
			for (m = 0; m < (n_end > 8 ? 8 : n_end); m++) {
				int inc = (ctx_set << 2) + greater1_ctx;
				g1_flag[m] = coeff_abs_level_greater1_flag_decode(cabac, inc);
				if (g1_flag[m]) {
					greater1_ctx = 0;
					if (first_g1_idx == -1)
						first_g1_idx = m;
				} else if (greater1_ctx > 0 && greater1_ctx < 3) {
					greater1_ctx++;
				}
			}

			first_nz_pos = sig_flag_idx[n_end - 1];
			sign_hidden = (last_nz_pos - first_nz_pos >= 4);

			/* Decode greater2 flag (at most 1) */
			if (first_g1_idx != -1) {
				g1_flag[first_g1_idx] +=
					coeff_abs_level_greater2_flag_decode(cabac, ctx_set);
			}

			/* Decode sign bits */
			{
				int nb_sign = sign_hidden ? (n_end - 1) : n_end;
				for (m = 0; m < nb_sign; m++)
					signs[m] = coeff_sign_flag_decode(cabac);
			}

			/* Decode abs_level_remaining and fill coeffs */
			{
				int sum_abs = 0;

				for (m = 0; m < n_end; m++) {
					s64 level;
					n = sig_flag_idx[m];
					DVEL_GET_COORD(scan_x_off, scan_y_off, x_cg, y_cg, n);

					if (m < 8) {
						level = 1 + g1_flag[m];
						if (level == ((m == first_g1_idx) ? 3 : 2)) {
							int rem = coeff_abs_level_remaining_decode(cabac, c_rice);
							level += rem;
							if (rem > (3 << c_rice) - 1)
								c_rice = min(c_rice + 1, 4);
						}
					} else {
						int rem = coeff_abs_level_remaining_decode(cabac, c_rice);
						level = 1 + rem;
						if (rem > (3 << c_rice) - 1)
							c_rice = min(c_rice + 1, 4);
					}

					/* Sign bit */
					if (sign_hidden) {
						sum_abs += level;
						if (m < n_end - 1 && signs[m])
							level = -level;
						else if (m == n_end - 1 && (sum_abs & 1))
							level = -level;
					} else {
						if (signs[m])
							level = -level;
					}

					coeffs[y_c * trafo_size + x_c] = (s16)level;
				}
			}
		}
	}

	/* --- Dequantize --- */
	if (c_idx == 0) {
		qp = ctx->qp_y;
	} else if (c_idx == 1) {
		qp = ctx->qp_y + ctx->pps.cb_qp_offset;
		if (qp < 0)
			qp = 0;
		else if (qp > 51)
			qp = 51;
		qp = dvel_chroma_qp(qp, 0);
	} else {
		qp = ctx->qp_y + ctx->pps.cr_qp_offset;
		if (qp < 0)
			qp = 0;
		else if (qp > 51)
			qp = 51;
		qp = dvel_chroma_qp(qp, 1);
	}
	qp += ctx->sps.bit_depth - 8;
	shift = ctx->sps.bit_depth + log2_trafo_size - 5;
	add = 1 << (shift - 1);
	scale = level_scale[rem6[qp]] << div6[qp];

	for (i = 0; i < trafo_size * trafo_size; i++) {
		s64 val = (s64)coeffs[i] * scale + add;
		val >>= shift;
		if (val > 32767)
			val = 32767;
		else if (val < -32768)
			val = -32768;
		coeffs[i] = (s16)val;
	}
}

/* ================================================================
 * CTU/CU/TU tree decode functions
 * ================================================================ */

/* Forward declarations */
static void dvel_idct(s16 *coeffs, int log2_trafo_size);

/* CABAC context decode helpers for split/CBF */

static int split_coding_unit_decode(struct dvel_cabac *c)
{
	return cabac_decode_bin(c, 0);
}

static int split_transform_flag_decode(struct dvel_cabac *c, int log2_trafo_size)
{
	int ctx = 1 + (log2_trafo_size - 2);
	return cabac_decode_bin(c, ctx);
}

static int cbf_luma_decode(struct dvel_cabac *c)
{
	return cabac_decode_bin(c, 114);
}

static int cbf_cb_cr_decode(struct dvel_cabac *c, int idx, int trafo_depth)
{
	return cabac_decode_bin(c, 115 + idx + (trafo_depth > 0 ? 1 : 0));
}

/* Add decoded residual coeffs to the output picture buffer.
 * For luma (c_idx=0): full size TU at (x0,y0).
 * For chroma (c_idx=1,2): half-size TU (420 subsampled) at (x0/2, y0/2).
 */
static void add_residual_to_pic(struct dvel_pic *pic, s16 *coeffs,
				int x0, int y0, int log2_size, int c_idx)
{
	int stride = pic->stride;
	int size = 1 << log2_size;
	int i, j;

	if (c_idx == 0) {
		s16 *buf = pic->y;
		for (j = 0; j < size; j++) {
			int y = y0 + j;
			if (y >= pic->height)
				break;
			for (i = 0; i < size; i++) {
				int x = x0 + i;
				if (x >= pic->width)
					break;
				buf[y * stride + x] += coeffs[j * size + i];
			}
		}
	} else if (c_idx == 1) {
		/* Cb */
		s16 *buf = pic->u;
		int x1 = x0 >> 1, y1 = y0 >> 1;
		int cw = pic->width >> 1;
		int ch = pic->height >> 1;
		for (j = 0; j < size; j++) {
			int y = y1 + j;
			if (y >= ch)
				break;
			for (i = 0; i < size; i++) {
				int x = x1 + i;
				if (x >= cw)
					break;
				buf[y * (stride >> 1) + x] += coeffs[j * size + i];
			}
		}
	} else {
		/* Cr */
		s16 *buf = pic->v;
		int x1 = x0 >> 1, y1 = y0 >> 1;
		int cw = pic->width >> 1;
		int ch = pic->height >> 1;
		for (j = 0; j < size; j++) {
			int y = y1 + j;
			if (y >= ch)
				break;
			for (i = 0; i < size; i++) {
				int x = x1 + i;
				if (x >= cw)
					break;
				buf[y * (stride >> 1) + x] += coeffs[j * size + i];
			}
		}
	}
}

/* Decode transform unit at leaf TU.
 * Calls residual_coding + IDCT + add-to-output.
 */
static int decode_tu_leaf(struct dvel_ctx *ctx, int x0, int y0,
			  int log2_trafo_size, int depth)
{
	int cbf_luma;
	int chroma_log2 = log2_trafo_size -
			   ((ctx->sps.chroma_format_idc == DVEL_CHROMA_420) ? 1 : 0);
	int cbf_cb = 0, cbf_cr = 0;
	s16 coeffs[1024]; /* 32x32 max */

	if (ctx->sps.chroma_format_idc != DVEL_CHROMA_MONO) {
		cbf_cb = cbf_cb_cr_decode(&ctx->cabac, 0, depth);
		cbf_cr = cbf_cb_cr_decode(&ctx->cabac, 1, depth);
	}

	if (depth == 0 || cbf_cb || cbf_cr)
		cbf_luma = 1;
	else
		cbf_luma = cbf_luma_decode(&ctx->cabac);

	if (cbf_luma) {
		residual_coding(ctx, coeffs, log2_trafo_size, 0);
		dvel_idct(coeffs, log2_trafo_size);
		add_residual_to_pic(ctx->pic, coeffs, x0, y0,
				    log2_trafo_size, 0);
	}

	if (cbf_cb) {
		residual_coding(ctx, coeffs, chroma_log2, 1);
		dvel_idct(coeffs, chroma_log2);
		add_residual_to_pic(ctx->pic, coeffs, x0, y0,
				    chroma_log2, 1);
	}
	if (cbf_cr) {
		residual_coding(ctx, coeffs, chroma_log2, 2);
		dvel_idct(coeffs, chroma_log2);
		add_residual_to_pic(ctx->pic, coeffs, x0, y0,
				    chroma_log2, 2);
	}
	return 0;
}

/* Decode transform tree recursively.
 * Calls decode_tu_leaf at leaf positions.
 */
static int decode_tu_tree(struct dvel_ctx *ctx, int x0, int y0,
			  int log2_trafo_size, int depth)
{
	int split;
	int log2_min_tb = ctx->sps.log2_min_tb_size;
	int max_tb_depth = ctx->sps.max_tb_depth;

	if (depth < max_tb_depth && log2_trafo_size > log2_min_tb) {
		split = split_transform_flag_decode(&ctx->cabac,
						    log2_trafo_size);
		if (split) {
			int half = 1 << (log2_trafo_size - 1);
			int new_log2 = log2_trafo_size - 1;
			decode_tu_tree(ctx, x0, y0, new_log2, depth + 1);
			decode_tu_tree(ctx, x0 + half, y0, new_log2,
				       depth + 1);
			decode_tu_tree(ctx, x0, y0 + half, new_log2,
				       depth + 1);
			decode_tu_tree(ctx, x0 + half, y0 + half,
				       new_log2, depth + 1);
			return 0;
		}
	}

	return decode_tu_leaf(ctx, x0, y0, log2_trafo_size, depth);
}

/* Decode a coding unit (leaf of CU quadtree).
 * In EL, we only need the TU partition structure, no prediction.
 */
static int decode_cu(struct dvel_ctx *ctx, int x0, int y0, int log2_cu_size)
{
	int log2_tu_root;
	int max_trafo = ctx->sps.log2_max_trafo_size;

	log2_tu_root = min(log2_cu_size, max_trafo);
	return decode_tu_tree(ctx, x0, y0, log2_tu_root, 0);
}

/* Decode coding quadtree.
 * Recursively decodes split flags and processes leaf CUs.
 */
static int decode_coding_quadtree(struct dvel_ctx *ctx,
				  int x0, int y0, int log2_size)
{
	int log2_min_cb = ctx->sps.log2_min_cb_size;

	if (log2_size > log2_min_cb) {
		int split = split_coding_unit_decode(&ctx->cabac);
		if (split) {
			int half = 1 << (log2_size - 1);
			int new_log2 = log2_size - 1;
			decode_coding_quadtree(ctx, x0, y0, new_log2);
			decode_coding_quadtree(ctx, x0 + half, y0, new_log2);
			decode_coding_quadtree(ctx, x0, y0 + half, new_log2);
			decode_coding_quadtree(ctx, x0 + half, y0 + half,
					      new_log2);
			return 0;
		}
	}

	return decode_cu(ctx, x0, y0, log2_size);
}

/* Decode all CTUs in the frame */
static int decode_ctus(struct dvel_ctx *ctx)
{
	int ctb_size = 1 << ctx->sps.log2_ctb_size;
	int width = ctx->width;
	int height = ctx->height;
	int ctb_x, ctb_y;

	for (ctb_y = 0; ctb_y * ctb_size < height; ctb_y++) {
		for (ctb_x = 0; ctb_x * ctb_size < width; ctb_x++) {
			int x0 = ctb_x * ctb_size;
			int y0 = ctb_y * ctb_size;
			int ret;

			ret = decode_coding_quadtree(ctx, x0, y0,
						     ctx->sps.log2_ctb_size);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

/* ================================================================
 * IDCT transforms (from FFmpeg dsp_template.c / dsp.c)
 * ================================================================ */

/* HEVC transform matrix (32x32) from FFmpeg hevc/dsp.c */
static const s8 dvel_transform[32][32] = {
	{ 64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
	  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64 },
	{ 90,  90,  88,  85,  82,  78,  73,  67,  61,  54,  46,  38,  31,  22,  13,   4,
	  -4, -13, -22, -31, -38, -46, -54, -61, -67, -73, -78, -82, -85, -88, -90, -90 },
	{ 90,  87,  80,  70,  57,  43,  25,   9,  -9, -25, -43, -57, -70, -80, -87, -90,
	 -90, -87, -80, -70, -57, -43, -25,  -9,   9,  25,  43,  57,  70,  80,  87,  90 },
	{ 90,  82,  67,  46,  22,  -4, -31, -54, -73, -85, -90, -88, -78, -61, -38, -13,
	  13,  38,  61,  78,  88,  90,  85,  73,  54,  31,   4, -22, -46, -67, -82, -90 },
	{ 89,  75,  50,  18, -18, -50, -75, -89, -89, -75, -50, -18,  18,  50,  75,  89,
	  89,  75,  50,  18, -18, -50, -75, -89, -89, -75, -50, -18,  18,  50,  75,  89 },
	{ 88,  67,  31, -13, -54, -82, -90, -78, -46,  -4,  38,  73,  90,  85,  61,  22,
	 -22, -61, -85, -90, -73, -38,   4,  46,  78,  90,  82,  54,  13, -31, -67, -88 },
	{ 87,  57,   9, -43, -80, -90, -70, -25,  25,  70,  90,  80,  43,  -9, -57, -87,
	 -87, -57,  -9,  43,  80,  90,  70,  25, -25, -70, -90, -80, -43,   9,  57,  87 },
	{ 85,  46, -13, -67, -90, -73, -22,  38,  82,  88,  54,  -4, -61, -90, -78, -31,
	  31,  78,  90,  61,   4, -54, -88, -82, -38,  22,  73,  90,  67,  13, -46, -85 },
	{ 83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83,
	  83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83 },
	{ 82,  22, -54, -90, -61,  13,  78,  85,  31, -46, -90, -67,   4,  73,  88,  38,
	 -38, -88, -73,  -4,  67,  90,  46, -31, -85, -78, -13,  61,  90,  54, -22, -82 },
	{ 80,   9, -70, -87, -25,  57,  90,  43, -43, -90, -57,  25,  87,  70,  -9, -80,
	 -80,  -9,  70,  87,  25, -57, -90, -43,  43,  90,  57, -25, -87, -70,   9,  80 },
	{ 78,  -4, -82, -73,  13,  85,  67, -22, -88, -61,  31,  90,  54, -38, -90, -46,
	  46,  90,  38, -54, -90, -31,  61,  88,  22, -67, -85, -13,  73,  82,   4, -78 },
	{ 75, -18, -89, -50,  50,  89,  18, -75, -75,  18,  89,  50, -50, -89, -18,  75,
	  75, -18, -89, -50,  50,  89,  18, -75, -75,  18,  89,  50, -50, -89, -18,  75 },
	{ 73, -31, -90, -22,  78,  67, -38, -90, -13,  82,  61, -46, -88,  -4,  85,  54,
	 -54, -85,   4,  88,  46, -61, -82,  13,  90,  38, -67, -78,  22,  90,  31, -73 },
	{ 70, -43, -87,   9,  90,  25, -80, -57,  57,  80, -25, -90,  -9,  87,  43, -70,
	 -70,  43,  87,  -9, -90, -25,  80,  57, -57, -80,  25,  90,   9, -87, -43,  70 },
	{ 67, -54, -78,  38,  85, -22, -90,   4,  90,  13, -88, -31,  82,  46, -73, -61,
	  61,  73, -46, -82,  31,  88, -13, -90,  -4,  90,  22, -85, -38,  78,  54, -67 },
	{ 64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,
	  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64 },
	{ 61, -73, -46,  82,  31, -88, -13,  90,  -4, -90,  22,  85, -38, -78,  54,  67,
	 -67, -54,  78,  38, -85, -22,  90,   4, -90,  13,  88, -31, -82,  46,  73, -61 },
	{ 57, -80, -25,  90,  -9, -87,  43,  70, -70, -43,  87,   9, -90,  25,  80, -57,
	 -57,  80,  25, -90,   9,  87, -43, -70,  70,  43, -87,  -9,  90, -25, -80,  57 },
	{ 54, -85,  -4,  88, -46, -61,  82,  13, -90,  38,  67, -78, -22,  90, -31, -73,
	  73,  31, -90,  22,  78, -67, -38,  90, -13, -82,  61,  46, -88,   4,  85, -54 },
	{ 50, -89,  18,  75, -75, -18,  89, -50, -50,  89, -18, -75,  75,  18, -89,  50,
	  50, -89,  18,  75, -75, -18,  89, -50, -50,  89, -18, -75,  75,  18, -89,  50 },
	{ 46, -90,  38,  54, -90,  31,  61, -88,  22,  67, -85,  13,  73, -82,   4,  78,
	 -78,  -4,  82, -73, -13,  85, -67, -22,  88, -61, -31,  90, -54, -38,  90, -46 },
	{ 43, -90,  57,  25, -87,  70,   9, -80,  80,  -9, -70,  87, -25, -57,  90, -43,
	 -43,  90, -57, -25,  87, -70,  -9,  80, -80,   9,  70, -87,  25,  57, -90,  43 },
	{ 38, -88,  73,  -4, -67,  90, -46, -31,  85, -78,  13,  61, -90,  54,  22, -82,
	  82, -22, -54,  90, -61, -13,  78, -85,  31,  46, -90,  67,   4, -73,  88, -38 },
	{ 36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36,
	  36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36 },
	{ 31, -78,  90, -61,   4,  54, -88,  82, -38, -22,  73, -90,  67, -13, -46,  85,
	 -85,  46,  13, -67,  90, -73,  22,  38, -82,  88, -54,  -4,  61, -90,  78, -31 },
	{ 25, -70,  90, -80,  43,   9, -57,  87, -87,  57,  -9, -43,  80, -90,  70, -25,
	 -25,  70, -90,  80, -43,  -9,  57, -87,  87, -57,   9,  43, -80,  90, -70,  25 },
	{ 22, -61,  85, -90,  73, -38,  -4,  46, -78,  90, -82,  54, -13, -31,  67, -88,
	  88, -67,  31,  13, -54,  82, -90,  78, -46,   4,  38, -73,  90, -85,  61, -22 },
	{ 18, -50,  75, -89,  89, -75,  50, -18, -18,  50, -75,  89, -89,  75, -50,  18,
	  18, -50,  75, -89,  89, -75,  50, -18, -18,  50, -75,  89, -89,  75, -50,  18 },
	{ 13, -38,  61, -78,  88, -90,  85, -73,  54, -31,   4,  22, -46,  67, -82,  90,
	 -90,  82, -67,  46, -22,  -4,  31, -54,  73, -85,  90, -88,  78, -61,  38, -13 },
	{  9, -25,  43, -57,  70, -80,  87, -90,  90, -87,  80, -70,  57, -43,  25,  -9,
	  -9,  25, -43,  57, -70,  80, -87,  90, -90,  87, -80,  70, -57,  43, -25,   9 },
	{  4, -13,  22, -31,  38, -46,  54, -61,  67, -73,  78, -82,  85, -88,  90, -90,
	  90, -90,  88, -85,  82, -78,  73, -67,  61, -54,  46, -38,  31, -22,  13,  -4 },
};

/* TR_4: 4-point 1D transform (hardcoded with 4x4 DCT matrix) */
#define DVEL_TR_4(dst, src, dstep, sstep, shift, add) \
	do { \
		int e0 = 64 * src[0 * sstep] + 64 * src[2 * sstep]; \
		int e1 = 64 * src[0 * sstep] - 64 * src[2 * sstep]; \
		int o0 = 83 * src[1 * sstep] + 36 * src[3 * sstep]; \
		int o1 = 36 * src[1 * sstep] - 83 * src[3 * sstep]; \
		dst[0 * dstep] = ((e0 + o0) + add) >> shift; \
		dst[1 * dstep] = ((e1 + o1) + add) >> shift; \
		dst[2 * dstep] = ((e1 - o1) + add) >> shift; \
		dst[3 * dstep] = ((e0 - o0) + add) >> shift; \
	} while (0)

/* TR_8: 8-point 1D transform using transform[][] table + TR_4 */
#define DVEL_TR_8(dst, src, dstep, sstep, shift, add, end) \
	do { \
		int i_, j_; \
		int e_8[4]; \
		int o_8[4] = { 0 }; \
		for (i_ = 0; i_ < 4; i_++) \
			for (j_ = 1; j_ < end; j_ += 2) \
				o_8[i_] += dvel_transform[4 * j_][i_] * src[j_ * sstep]; \
		DVEL_TR_4(e_8, src, 1, 2 * sstep, 0, 0); \
		for (i_ = 0; i_ < 4; i_++) { \
			dst[i_ * dstep] = ((e_8[i_] + o_8[i_]) + add) >> shift; \
			dst[(7 - i_) * dstep] = ((e_8[i_] - o_8[i_]) + add) >> shift; \
		} \
	} while (0)

/* TR_16: 16-point 1D transform */
#define DVEL_TR_16(dst, src, dstep, sstep, shift, add, end) \
	do { \
		int i_, j_; \
		int e_16[8]; \
		int o_16[8] = { 0 }; \
		for (i_ = 0; i_ < 8; i_++) \
			for (j_ = 1; j_ < end; j_ += 2) \
				o_16[i_] += dvel_transform[2 * j_][i_] * src[j_ * sstep]; \
		DVEL_TR_8(e_16, src, 1, 2 * sstep, 0, 0, 8); \
		for (i_ = 0; i_ < 8; i_++) { \
			dst[i_ * dstep] = ((e_16[i_] + o_16[i_]) + add) >> shift; \
			dst[(15 - i_) * dstep] = ((e_16[i_] - o_16[i_]) + add) >> shift; \
		} \
	} while (0)

/* TR_32: 32-point 1D transform */
#define DVEL_TR_32(dst, src, dstep, sstep, shift, add, end) \
	do { \
		int i_, j_; \
		int e_32[16]; \
		int o_32[16] = { 0 }; \
		for (i_ = 0; i_ < 16; i_++) \
			for (j_ = 1; j_ < end; j_ += 2) \
				o_32[i_] += dvel_transform[j_][i_] * src[j_ * sstep]; \
		DVEL_TR_16(e_32, src, 1, 2 * sstep, 0, 0, end / 2); \
		for (i_ = 0; i_ < 16; i_++) { \
			dst[i_ * dstep] = ((e_32[i_] + o_32[i_]) + add) >> shift; \
			dst[(31 - i_) * dstep] = ((e_32[i_] - o_32[i_]) + add) >> shift; \
		} \
	} while (0)

/* 2D IDCT: column pass then row pass */
static void dvel_idct_4x4(s16 *coeffs)
{
	int i;

	for (i = 0; i < 4; i++)
		DVEL_TR_4(coeffs + i, coeffs + i, 4, 4, 7, 32);
	for (i = 0; i < 4; i++)
		DVEL_TR_4(coeffs + i * 4, coeffs + i * 4, 1, 1, 12, 2048);
}

static void dvel_idct_8x8(s16 *coeffs)
{
	int i;

	for (i = 0; i < 8; i++)
		DVEL_TR_8(coeffs + i, coeffs + i, 8, 8, 7, 64, 8);
	for (i = 0; i < 8; i++)
		DVEL_TR_8(coeffs + i * 8, coeffs + i * 8, 1, 1, 12, 2048, 8);
}

static void dvel_idct_16x16(s16 *coeffs)
{
	int i;

	for (i = 0; i < 16; i++)
		DVEL_TR_16(coeffs + i, coeffs + i, 16, 16, 7, 64, 16);
	for (i = 0; i < 16; i++)
		DVEL_TR_16(coeffs + i * 16, coeffs + i * 16, 1, 1, 12, 2048, 16);
}

static void dvel_idct_32x32(s16 *coeffs)
{
	int i;

	for (i = 0; i < 32; i++)
		DVEL_TR_32(coeffs + i, coeffs + i, 32, 32, 7, 64, 32);
	for (i = 0; i < 32; i++)
		DVEL_TR_32(coeffs + i * 32, coeffs + i * 32, 1, 1, 12, 2048, 32);
}

static void dvel_idct(s16 *coeffs, int log2_trafo_size)
{
	switch (log2_trafo_size) {
	case 2: dvel_idct_4x4(coeffs); break;
	case 3: dvel_idct_8x8(coeffs); break;
	case 4: dvel_idct_16x16(coeffs); break;
	case 5: dvel_idct_32x32(coeffs); break;
	}
}

/* ================================================================
 * Decoder entry point
 * ================================================================ */

int dvel_init(struct dvel_ctx *ctx, int width, int height, int bit_depth)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->width = width;
	ctx->height = height;
	ctx->bit_depth = bit_depth;

	ctx->sps.pic_width_in_luma_samples = width;
	ctx->sps.pic_height_in_luma_samples = height;
	ctx->sps.bit_depth = bit_depth;
	ctx->sps.log2_ctb_size = 6;
	ctx->sps.chroma_format_idc = DVEL_CHROMA_420;
	ctx->sps_valid = true;

	/* Allocate output frame buffer.
	 * Called from dvel_global_init which may run in ISR context,
	 * so use GFP_ATOMIC.
	 */
	ctx->pic = kzalloc(sizeof(struct dvel_pic), GFP_ATOMIC);
	if (!ctx->pic)
		return -ENOMEM;

	ctx->pic->width = width;
	ctx->pic->height = height;
	ctx->pic->stride = ALIGN(width, 64);
	ctx->pic->bit_depth = bit_depth;

	ctx->pic->y = kzalloc(ctx->pic->stride * height * 2, GFP_ATOMIC);
	if (!ctx->pic->y) {
		kfree(ctx->pic);
		ctx->pic = NULL;
		return -ENOMEM;
	}

	/* Chroma buffers (4:2:0 subsampled, u16 per sample) */
	{
		int c_stride = ctx->pic->stride >> 1;
		int c_height = ALIGN(height, 2) >> 1;
		ctx->pic->u = kzalloc(c_stride * c_height * 2, GFP_ATOMIC);
		ctx->pic->v = kzalloc(c_stride * c_height * 2, GFP_ATOMIC);
		if (!ctx->pic->u || !ctx->pic->v) {
			kfree(ctx->pic->u);
			kfree(ctx->pic->y);
			kfree(ctx->pic);
			ctx->pic = NULL;
			return -ENOMEM;
		}
	}

	ctx->frame_count = 0;

	return 0;
}

void dvel_exit(struct dvel_ctx *ctx)
{
	if (ctx->pic) {
		kfree(ctx->pic->y);
		kfree(ctx->pic->u);
		kfree(ctx->pic->v);
		kfree(ctx->pic);
		ctx->pic = NULL;
	}
}

int dvel_decode(struct dvel_ctx *ctx, const u8 *data, int size, int poc)
{
	int nal_type, layer_id;
	int offset;

	if (size < 4)
		return -1;

	offset = parse_nal_header(data, size, &nal_type, &layer_id);
	if (offset < 0)
		return -1;

	switch (nal_type) {
	case DVEL_NAL_SPS:
		return parse_sps(ctx, data + offset, size - offset);

	case DVEL_NAL_PPS:
		return parse_pps(ctx, data + offset, size - offset);

	case DVEL_NAL_EOS:
		return 0;

	case DVEL_NAL_TRAIL_N:
	case DVEL_NAL_TRAIL_R:
	{
		/* Slice NAL - decode residual */
		const u8 *nal_data = data + offset;
		int nal_size = size - offset;
		u8 rbsp[4096];
		int rbsp_size, slice_end_bits, cabac_offset_bytes;
		int init_type, slice_type, ret;

		if (!ctx->sps_valid || !ctx->pps_valid)
			return -1;

		/* Remove emulation prevention from the NAL data */
		if (nal_size > (int)sizeof(rbsp))
			return -1;
		remove_emul_3bytes(nal_data, nal_size, rbsp, &rbsp_size);

		/* Parse slice header from RBSP */
		slice_end_bits = parse_slice_header_rbsp(ctx, rbsp, rbsp_size,
							  &slice_type);
		if (slice_end_bits < 0)
			return -1;

		ctx->nal_unit_type = nal_type;

		/* Determine CABAC init type */
		switch (slice_type) {
		case 2: /* I slice */
			init_type = 0;
			break;
		case 1: /* P slice */
			init_type = 1;
			break;
		case 0: /* B slice */
		default:
			init_type = 2;
			break;
		}

		/* CABAC data starts after byte-aligned slice header end.
		 * The cabac_alignment_one_bit (always '1') and alignment
		 * padding are consumed by byte alignment.
		 */
		cabac_offset_bytes = (slice_end_bits + 7) >> 3;
		if (cabac_offset_bytes >= rbsp_size)
			return -1;

		/* Initialize CABAC */
		cabac_init(&ctx->cabac,
			   rbsp + cabac_offset_bytes,
			   rbsp_size - cabac_offset_bytes,
			   init_type,
			   ctx->slice_qp);

		/* Clear residual buffers before decoding */
		memset(ctx->pic->y, 0, ctx->pic->stride * ctx->height * 2);
		if (ctx->sps.chroma_format_idc != DVEL_CHROMA_MONO) {
			int c_stride = ctx->pic->stride >> 1;
			int c_height = (ALIGN(ctx->height, 2)) >> 1;
			memset(ctx->pic->u, 0, c_stride * c_height * 2);
			memset(ctx->pic->v, 0, c_stride * c_height * 2);
		}

		/* Decode all CTUs in the frame */
		ret = decode_ctus(ctx);
		if (ret < 0) {
			ctx->error_count++;
			return ret;
		}

		ctx->poc = poc;
		ctx->frame_count++;
		return 0;
	}

	case DVEL_NAL_SEI_PREFIX:
	case DVEL_NAL_SEI_SUFFIX:
		return 0;

	default:
		return 0;
	}
}

/* ================================================================
 * VFM provider registration
 * ================================================================ */

#include <linux/kfifo.h>
#include <linux/amlogic/media/vfm/vframe.h>
#include <linux/amlogic/media/vfm/vframe_provider.h>
#include <linux/amlogic/media/vfm/vframe_receiver.h>

#define DVEL_PROVIDER_NAME "dveldec"
#define DVEL_FRAME_POOL_SIZE 4

struct dvel_vf_pool {
	struct vframe_s vf[DVEL_FRAME_POOL_SIZE];
	struct dvel_pic pic[DVEL_FRAME_POOL_SIZE];
};
static struct dvel_vf_pool *g_pool;
static struct vframe_provider_s g_dvel_prov;
static DEFINE_KFIFO(g_ready_q, struct vframe_s *, DVEL_FRAME_POOL_SIZE);
static DEFINE_KFIFO(g_free_q, int, DVEL_FRAME_POOL_SIZE);

static struct dvel_ctx *g_dvel_ctx;

static struct vframe_s *dvel_vf_peek(void *op_arg)
{
	struct vframe_s *vf;

	if (kfifo_peek(&g_ready_q, &vf))
		return vf;
	return NULL;
}

static struct vframe_s *dvel_vf_get(void *op_arg)
{
	struct vframe_s *vf;

	if (kfifo_get(&g_ready_q, &vf))
		return vf;
	return NULL;
}

static void dvel_vf_put(struct vframe_s *vf, void *op_arg)
{
	int i;

	for (i = 0; i < DVEL_FRAME_POOL_SIZE; i++) {
		if (&g_pool->vf[i] == vf) {
			kfifo_put(&g_free_q, i);
			return;
		}
	}
}

static int dvel_event_cb(int type, void *data, void *private_data)
{
	return 0;
}

static int dvel_vf_states(struct vframe_states *states, void *op_arg)
{
	states->vf_pool_size = DVEL_FRAME_POOL_SIZE;
	states->buf_free_num = g_pool ? kfifo_len(&g_free_q) : 0;
	states->buf_avail_num = kfifo_len(&g_ready_q);
	states->buf_recycle_num = DVEL_FRAME_POOL_SIZE -
				  states->buf_free_num -
				  states->buf_avail_num;
	return 0;
}

static const struct vframe_operations_s dvel_vf_ops = {
	.peek     = dvel_vf_peek,
	.get      = dvel_vf_get,
	.put      = dvel_vf_put,
	.event_cb = dvel_event_cb,
	.vf_states = dvel_vf_states,
};

static int dvel_provider_init(void)
{
	int i, ret;

	g_pool = kzalloc(sizeof(*g_pool), GFP_ATOMIC);
	if (!g_pool)
		return -ENOMEM;

	/* Pre-allocate picture buffers for pool */
	for (i = 0; i < DVEL_FRAME_POOL_SIZE; i++) {
		struct dvel_pic *pic = &g_pool->pic[i];

		pic->stride = 0; /* set when used */
		pic->y = NULL;
		pic->u = NULL;
		pic->v = NULL;
	}

	for (i = 0; i < DVEL_FRAME_POOL_SIZE; i++)
		kfifo_put(&g_free_q, i);

	vf_provider_init(&g_dvel_prov, DVEL_PROVIDER_NAME,
			 &dvel_vf_ops, NULL);
	ret = vf_reg_provider(&g_dvel_prov);
	if (ret < 0) {
		pr_err("dvel: failed to register VFM provider\n");
		kfree(g_pool);
		g_pool = NULL;
		return ret;
	}

	vf_notify_receiver(DVEL_PROVIDER_NAME,
			   VFRAME_EVENT_PROVIDER_START, NULL);
	pr_info("dvel: registered provider \"%s\"\n", DVEL_PROVIDER_NAME);
	return 0;
}

static void dvel_provider_exit(void)
{
	int i;

	if (g_pool) {
		vf_unreg_provider(&g_dvel_prov);
		for (i = 0; i < DVEL_FRAME_POOL_SIZE; i++) {
			kfree(g_pool->pic[i].y);
			kfree(g_pool->pic[i].u);
			kfree(g_pool->pic[i].v);
		}
		kfree(g_pool);
		g_pool = NULL;
	}
}

/* Allocate pic buffers for a pool entry (first use or after reinit) */
static int dvel_pool_pic_alloc(struct dvel_pic *pic, int width, int height,
				int bit_depth)
{
	int stride = ALIGN(width, 64);
	int c_stride = stride >> 1;
	int c_height = ALIGN(height, 2) >> 1;

	pic->y = kzalloc(stride * height * 2, GFP_ATOMIC);
	if (!pic->y)
		return -ENOMEM;
	pic->u = kzalloc(c_stride * c_height * 2, GFP_ATOMIC);
	if (!pic->u)
		goto fail;
	pic->v = kzalloc(c_stride * c_height * 2, GFP_ATOMIC);
	if (!pic->v)
		goto fail;

	pic->width = width;
	pic->height = height;
	pic->stride = stride;
	pic->bit_depth = bit_depth;
	return 0;

fail:
	kfree(pic->y);
	kfree(pic->u);
	pic->y = NULL;
	pic->u = NULL;
	return -ENOMEM;
}

/* Queue the decoded frame from ctx into the VFM provider's ready queue */
static int dvel_frame_ready(struct dvel_ctx *ctx)
{
	struct vframe_s *vf;
	struct dvel_pic *pic;
	int idx;

	if (!g_pool)
		return -ENODEV;

	if (!kfifo_get(&g_free_q, &idx)) {
		pr_err_once("dvel: pool full, dropping EL frame poc %d\n",
			    ctx->poc);
		return -EAGAIN;
	}

	vf = &g_pool->vf[idx];
	pic = &g_pool->pic[idx];
	memset(vf, 0, sizeof(*vf));

	pic->poc = ctx->poc;

	/* Pool buffers are pre-allocated in dvel_global_init */
	if (!pic->y) {
		/* Fallback: should not happen after init pre-allocates */
		if (dvel_pool_pic_alloc(pic, ctx->width, ctx->height,
					ctx->bit_depth) < 0) {
			kfifo_put(&g_free_q, idx);
			return -ENOMEM;
		}
	}

	/* Copy decoded data from ctx to pool pic */
	memcpy(pic->y, ctx->pic->y,
	       pic->stride * ctx->height * 2);
	if (ctx->sps.chroma_format_idc != DVEL_CHROMA_MONO) {
		int c_stride = pic->stride >> 1;
		int c_height = ALIGN(ctx->height, 2) >> 1;
		memcpy(pic->u, ctx->pic->u, c_stride * c_height * 2);
		memcpy(pic->v, ctx->pic->v, c_stride * c_height * 2);
	}

	vf->private_data = pic;
	vf->width = ctx->width;
	vf->height = ctx->height;
	vf->type = VIDTYPE_PROGRESSIVE;
	vf->bitdepth = ctx->bit_depth;
	vf->pts_us64 = ctx->poc;
	vf->dv_input = true;

	kfifo_put(&g_ready_q, vf);
	vf_notify_receiver(DVEL_PROVIDER_NAME,
			   VFRAME_EVENT_RECEIVER_FRAME_WAIT, NULL);
	return 0;
}

/* ================================================================
 * Module interface
 * ================================================================ */

int dvel_global_init(int width, int height, int bit_depth)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dvel_lock, flags);
	if (g_dvel_ctx) {
		/* Already initialized — reinit only if dimensions or bit depth changed */
		if (g_dvel_ctx->width == width &&
		    g_dvel_ctx->height == height &&
		    g_dvel_ctx->bit_depth == bit_depth) {
			spin_unlock_irqrestore(&dvel_lock, flags);
			return 0;
		}
		/* Dimensions changed: tear down and reinit */
		spin_unlock_irqrestore(&dvel_lock, flags);
		dvel_global_exit();
		spin_lock_irqsave(&dvel_lock, flags);
	}

	ret = dvel_provider_init();
	if (ret < 0) {
		spin_unlock_irqrestore(&dvel_lock, flags);
		return ret;
	}

	g_dvel_ctx = kzalloc(sizeof(struct dvel_ctx), GFP_ATOMIC);
	if (!g_dvel_ctx) {
		dvel_provider_exit();
		spin_unlock_irqrestore(&dvel_lock, flags);
		return -ENOMEM;
	}

	ret = dvel_init(g_dvel_ctx, width, height, bit_depth);
	if (ret < 0) {
		kfree(g_dvel_ctx);
		g_dvel_ctx = NULL;
		dvel_provider_exit();
		spin_unlock_irqrestore(&dvel_lock, flags);
		return ret;
	}

	/* Pre-allocate all pool picture buffers (process context or ISR-safe GFP_ATOMIC).
	 * This ensures dvel_frame_ready() never needs to allocate memory in the ISR.
	 */
	if (g_pool) {
		int i;
		for (i = 0; i < DVEL_FRAME_POOL_SIZE; i++) {
			if (!g_pool->pic[i].y) {
				if (dvel_pool_pic_alloc(&g_pool->pic[i],
							width, height, bit_depth) < 0) {
					pr_err("dvel: failed to pre-allocate pool[%d]\n", i);
				}
			}
		}
	}

	spin_unlock_irqrestore(&dvel_lock, flags);
	return 0;
}
EXPORT_SYMBOL(dvel_global_init);

int dvel_global_decode(const u8 *nal, int size, int poc)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dvel_lock, flags);
	if (!g_dvel_ctx) {
		spin_unlock_irqrestore(&dvel_lock, flags);
		return -EINVAL;
	}

	ret = dvel_decode(g_dvel_ctx, nal, size, poc);
	if (ret < 0) {
		spin_unlock_irqrestore(&dvel_lock, flags);
		return ret;
	}

	ret = dvel_frame_ready(g_dvel_ctx);
	spin_unlock_irqrestore(&dvel_lock, flags);
	return ret;
}
EXPORT_SYMBOL(dvel_global_decode);

void dvel_global_exit(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dvel_lock, flags);
	if (g_dvel_ctx) {
		dvel_exit(g_dvel_ctx);
		kfree(g_dvel_ctx);
		g_dvel_ctx = NULL;
	}
	spin_unlock_irqrestore(&dvel_lock, flags);

	/* Drain the ready queue before tearing down the provider.
	 * This prevents use-after-free if dovi.ko still holds a vframe reference.
	 */
	if (g_pool) {
		struct vframe_s *vf;
		while (kfifo_get(&g_ready_q, &vf)) {
			/* Return to free pool */
			int idx;
			for (idx = 0; idx < DVEL_FRAME_POOL_SIZE; idx++) {
				if (&g_pool->vf[idx] == vf) {
					kfifo_put(&g_free_q, idx);
					break;
				}
			}
		}
	}

	dvel_provider_exit();
}
EXPORT_SYMBOL(dvel_global_exit);
