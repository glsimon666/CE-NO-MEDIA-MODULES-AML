# dvel_swdec — Dolby Vision FEL Enhancement Layer Software Decoder

## 目的

在 S5 单核 HEVC 平台（vh265_fb.c）上实现纯内核空间的 DV FEL 增强层残差解码器，取代不可行的用户态方案（PTS 同步死锁）。

## 架构

```
Stream Buffer (EL NALs)
    │
    ▼
vh265_fb.c (bypass_dvenl=1)
    │ 检测 DVEL_NAL (0xA0) 后调用
    ▼
dvel_global_decode() ──► dvel_decode()
    │                        ├─ NAL/SPS/PPS 解析
    │                        ├─ Slice Header 解析
    │                        ├─ CABAC 初始化
    │                        └─ CTU 解码循环
    │                              ├─ split_cu_flag 四叉树
    │                              │   └─ split_transform_flag
    │                              │       └─ residual_coding() + IDCT + add_to_pic
    │                              ▼
    │                    ctx->pic (s16 残差数据)
    │
    ▼
dvel_frame_ready()
    │ 复制到 pool → vframe_s → kfifo(g_ready_q)
    │ vf_notify_receiver("dveldec", FRAME_WAIT)
    ▼
VFM Provider "dveldec"
    │ peek / get / put
    ▼
dovi.ko (Receiver "dvel")
    │ BL + EL merge
    ▼
Display
```

## 文件清单

| 文件 | 行数 | 说明 |
|------|------|------|
| `dvel_swdec.h` | ~150 | 数据结构定义、API 声明 |
| `dvel_swdec.c` | ~1990 | 解码器完整实现 |
| `dvel_cabac_init.h` | 3×179 | CABAC 初始化表（来自 FFmpeg） |
| `dvel_range_tab.h` | 4×64 | rangeTabLPS（HEVC spec Table 9-38） |
| `Makefile` | ~12 | 模块编译（CONFIG_AMLOGIC_MEDIA_VDEC_DVEL） |

## 核心实现要点

### 1. Bit Reader (`dvel_swdec.c:16-102`)

MSB-first get_bits 实现，支持：
- `br_get_bits(n)` — 读取 n 位
- `br_get_bits1()` — 读取 1 位
- `br_ue()` / `br_se()` — Exp-Golomb 解码
- `br_byte_aligned()` — 字节对齐检查
- `br_bit_pos()` — 返回已消耗比特数
- `remove_emul_3bytes()` — 去除 0x00 0x00 0x03 防竞争字节

### 2. CABAC 引擎 (`dvel_swdec.c:358-535`)

**位模型**: `CABAC_BITS=16`，与 FFmpeg 完全一致
- `low` 寄存器在 `range<<17` 标度下比较
- 初始化: `low` 加载前 2 字节 + `1<<9` 偏移

**状态编码**: `(mps<<7) | pStateIdx`（标准 HEVC 编码）
- `mps` 在 bit 7，`pStateIdx` 在 bits 0-5
- `trans_idx_mps[64]` / `trans_idx_lps[64]` — FFmpeg 状态转移表

**初始化公式** (FFmpeg `cabac_init_state`):
```c
m = ((init>>4)&0xF)*5 - 45;
n = ((init&0xF)<<3) - 16;
pre = 2*(((m*qp)>>4)+n) - 127;
if (pre >= 64) state = (1<<7) | (pre-64);
else state = pre;
```

**上下文索引**:
| 上下文 | 偏移 |
|--------|------|
| split_cu_flag | 0 |
| split_transform_flag | 1 + (log2_size - 2) |
| last_sig_coeff_x/y | 53 / 71 |
| sig_coeff_group | 89 |
| significant_coeff | 93 + scf_off |
| coeff_abs_level_greater1 | 137 |
| coeff_abs_level_greater2 | 161 |
| cbf_luma | 114 |
| cbf_cb | 115 |
| cbf_cr | 116 |

### 3. 残差解码 (`dvel_swdec.c:578-1138`)

**函数**: `residual_coding()`

流程:
1. `last_significant_coeff_x/y` — 前缀+后缀 CABAC 解码
2. CG (4×4) 循环 — `significant_coeff_group_flag`
3. `significant_coeff_flag` — 基于 `prev_sig` 上下文的逐系数解码
4. `coeff_abs_level_greater1_flag` / `greater2_flag` — 幅值等级
5. `coeff_abs_level_remaining` — Rice 码剩余幅值
6. `coeff_sign_flag` — 符号位（支持符号隐藏）
7. 反量化 — `level_scale[rem6[qp]] << div6[qp]`，FFmpeg 表

> **简化**: 移除 RDPCM、persistent_rice_adaptation、transform_skip（EL 不需要）

### 4. IDCT (`dvel_swdec.c:1367-1550`)

从 FFmpeg `dsp_template.c` / `dsp.c` 移植：

- **变换矩阵**: `dvel_transform[32][32]`，完整 32×32 DCT 系数（`dsp.c:27`）
- **1D 变换**: TR_4（硬编码 4 点）、TR_8/16/32（递归偶数分解 + 奇数矩阵乘法）
- **2D IDCT**: 列变换 → 行变换
  - 列 pass: shift=7, add=64 (8×8+)，32 (4×4)
  - 行 pass: shift=12, add=2048

### 5. CTU/CU/TU 树遍历 (`dvel_swdec.c:1139-1366`)

```c
decode_ctus()                    // 遍历所有 CTU
  └─ decode_coding_quadtree()    // split_cu_flag 递归
       └─ decode_cu()            // CU 叶节点
            └─ decode_tu_tree()  // split_transform_flag 递归
                 └─ decode_tu_leaf()  // TU 叶节点
                      ├─ cbf_luma (depth==0 时解码)
                      │   └─ residual_coding() + IDCT + add_residual
                      ├─ cbf_cb
                      │   └─ residual_coding() + IDCT + add_residual
                      └─ cbf_cr
                          └─ residual_coding() + IDCT + add_residual
```

### 6. VFM Provider (`dvel_swdec.c:1714-1986`)

- Provider 名称: `"dveldec"`（匹配 `vdec.h` 的 `VFM_DEC_DVEL_PROVIDER_NAME`）
- 接收器名称: `"dvel"`（dovi.ko 注册，无需修改）
- Frame Pool: 4 个 `vframe_s` + `dvel_pic` 预分配
- 队列: `kfifo(g_ready_q)` — 就绪帧; `kfifo(g_free_q)` — 可用索引
- 回调:
  - `peek` — 非破坏性读取就绪队列
  - `get` — 出队就绪帧
  - `put` — 归还帧到空闲池
  - `event_cb` — 暂空
  - `vf_states` — 报告缓冲池状态
- 通知: 解码完成后 `vf_notify_receiver("dveldec", VFRAME_EVENT_RECEIVER_FRAME_WAIT, NULL)`

### 7. 输出数据格式

`vframe_s->private_data` 指向 `dvel_pic`:

```c
struct dvel_pic {
    s16 *y;       // 亮度残差 (1920×1088 × 2 bytes)
    s16 *u;       // Cb 残差 (960×544 × 2 bytes)
    s16 *v;       // Cr 残差 (960×544 × 2 bytes)
    int width, height, stride;
    int poc;
    u8 bit_depth;
};
```

dovi.ko 通过 `private_data` 获取残差数据并与 BL 合并。

## 与 vh265_fb.c 集成

vh265_fb.c 中需要：

```c
// 1. 在 DV FEL 流开始时初始化
dvel_global_init(width, height, bit_depth);

// 2. 在 ucode 报告 DVEL_NAL (0xA0) 时传递 EL NAL
//    此时 bypass_dvenl=1, ucode 跳过了 EL 解码
dvel_global_decode(el_nal_data, el_nal_size, poc);

// 3. 在流停止/重置时清理
dvel_global_exit();
```

## 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 解码位置 | 纯内核（方案 C） | 用户态 PTS 同步不可行 |
| FFmpeg 引用方式 | 算法参考，不链接 | 内核模块不允许链接 libavcodec |
| CABAC 位模型 | `CABAC_BITS=16` | FFmpeg 兼容性，已验证 |
| 解码器形态 | 独立模块 dvel_swdec.ko | 不侵入 vh265_fb.c 主体逻辑 |
| VFM 通信 | provider "dveldec" + receiver "dvel" | 现有 dovi.ko 架构，无需修改 |
| 帧队列 | kfifo 4 帧池 | 1080p 残差，简单可靠 |
| 色度格式 | 4:2:0（s16 残差） | DV FEL 标准格式 |
| slice 支持 | 单 slice/帧 | EL 典型配置；多 slice 未实现 |

## 待办（Review 重点）

- [ ] **语法正确性**: 检查所有 switch/case 的 C90 兼容性（declaration after statement）
- [ ] **CABAC 精度**: 验证 `CABAC_BITS=16` 与 FFmpeg 的标度匹配（特别是 `range<<17` vs `low`）
- [ ] **上下文偏移**: 验证 `93 + scf_off` 在 `significant_coeff_flag_decode` 中的正确性
- [ ] **IDCT 蝶形**: TR_8/16/32 的 `dvel_transform` 索引（`4*j`、`2*j`、`j`）是否与 FFmpeg 一致
- [ ] **CBF 逻辑**: `decode_tu_leaf` 中 cbf_cb/cr 的 TU 级解码 vs HEVC 规范
- [ ] **sign_hidden**: `first_nz_pos` 与 `sum_abs` 奇偶校验逻辑
- [ ] **QP 处理**: 色度 QP 未独立计算（使用 luma QP），是否影响 EL 精度
- [ ] **边界检查**: `add_residual_to_pic` 的宽/高越界保护
- [ ] **内存释放**: `dvel_provider_exit` 中 pool pic buffer 的释放顺序
- [ ] **VFM 通知**: `vf_notify_receiver` 是否在正确时机调用
