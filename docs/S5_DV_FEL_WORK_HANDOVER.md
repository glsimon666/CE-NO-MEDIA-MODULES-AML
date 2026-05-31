# S5 芯片 DV FEL 解码修复 - 工作移交文档

## 一、项目背景

### 1.1 问题描述
S5 芯片播放 Dolby Vision FEL（Full Enhancement Layer）文件时出现 **"el not found"** 错误，无法正常解码。

### 1.2 问题分析
- SC2 芯片使用 h265 流解码器，支持双核硬件解码
- S5 芯片使用 h265_fb 帧解码器，原设计使用 `front_back_mode`（时分复用）
- S5 上禁用了 `front_back_mode` 以修复快退快进问题，但导致 DV FEL 解码失败

### 1.3 根本原因
h265_fb 解码器缺少完整的 DV 双核解码支持：
1. 缺少 `check_dv_flag` 配置读取
2. 缺少 `is_dv_flag` 初始化
3. 解码模式判断顺序不正确

---

## 二、已完成的代码修改

### 2.1 修改摘要

| 修改文件 | 修改类型 | 行号 | 状态 |
|---------|---------|------|------|
| `vh265_fb.c` | 结构体成员添加 | L1849-L1850 | ✅ 完成 |
| `vh265_fb.c` | 配置顺序调整 | L14733-L14754 | ✅ 完成 |
| `vh265_fb.c` | 配置读取添加 | L18606-L18608 | ✅ 完成 |
| `vh265_fb.c` | 初始化添加 | L17946-L17954 | ✅ 完成 |
| `amstream.c` | Slave Decoder 创建 | L1579-L1592 | ✅ 已有 |

### 2.2 详细修改内容

#### 修改1：结构体成员添加
**文件**: `drivers/frame_provider/decoder/h265_fb/vh265_fb.c`

```c
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
	unsigned char switch_dvlayer_flag;
	unsigned char no_switch_dvlayer_count;
	unsigned char bypass_dvenl_enable;
	unsigned char bypass_dvenl;
	unsigned char check_dv_flag;   // ✅ 新增：DV自动检测标志
	unsigned char is_dv_flag;      // ✅ 新增：是否是DV流标志
#endif
```

#### 修改2：配置顺序调整
**文件**: `drivers/frame_provider/decoder/h265_fb/vh265_fb.c`

```c
// ✅ 修改前：framebase 在前
if (!hevc->m_ins_flag)
    decode_mode = DECODE_MODE_SINGLE;
else if (vdec_frame_based(...))      // framebase
    decode_mode = DECODE_MODE_MULTI_FRAMEBASE;
else if (vdec->slave)               // DV双核
    decode_mode = DECODE_MODE_MULTI_DVBAL;

// ✅ 修改后：DV双核优先
if (!hevc->m_ins_flag)
    decode_mode = DECODE_MODE_SINGLE;
else if (vdec->slave)               // DV双核优先
    decode_mode = DECODE_MODE_MULTI_DVBAL;
else if (vdec->master)
    decode_mode = DECODE_MODE_MULTI_DVENL;
else if (vdec_frame_based(...))      // framebase在后
    decode_mode = DECODE_MODE_MULTI_FRAMEBASE;
```

#### 修改3：配置读取添加
**文件**: `drivers/frame_provider/decoder/h265_fb/vh265_fb.c`

```c
if (get_config_int(pdata->config, "parm_metadata_config_flag",
        &config_val) == 0) {
    hevc->high_bandwidth_flag = config_val & VDEC_CFG_FLAG_HIGH_BANDWIDTH;
    if (hevc->high_bandwidth_flag)
        hevc_print(hevc, 0, "high bandwidth\n");
    hevc->check_dv_flag = config_val & VDEC_CFG_FLAG_DV_AUTO_DETECT;  // ✅ 新增
    if (hevc->check_dv_flag)
        hevc_print(hevc, 0, "decode check dv\n");                      // ✅ 新增
}
```

#### 修改4：初始化添加
**文件**: `drivers/frame_provider/decoder/h265_fb/vh265_fb.c`

```c
hevc->fatal_error = 0;
hevc->show_frame_num = 0;
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
hevc->is_dv_flag = 0;  // ✅ 新增：DV标志初始化
#endif
hevc->frameinfo_enable = 1;
```

---

## 三、代码提交记录

| 提交哈希 | 描述 | 日期 | 分支 |
|---------|------|------|------|
| `f418a73d` | h265_fb: add check_dv_flag configuration for DV auto detection | 2026-05-31 | 20260530 |
| `69f3c9a9` | h265_fb: enable DV dual-core decoding for S5 chip | 2026-05-31 | 20260530 |

**远程仓库**: `glsimon/CE-NO-MEDIA-MODULES-AML`  
**分支**: `20260530`

---

## 四、测试计划

### 4.1 环境准备

#### 步骤1：编译内核
```bash
cd /path/to/coreelec
make ARCH=aarch64
```

#### 步骤2：部署到测试设备
- 将新内核镜像刷入 S5 设备
- 确保 `amvdec_h265_fb` 模块已集成

#### 步骤3：启用调试标志
```bash
# 启用双解码器模式
echo 256 > /sys/module/decoder_common/parameters/debugflags
```

### 4.2 测试用例

| 序号 | 测试项 | 预期结果 | 验证命令 |
|------|--------|---------|---------|
| 1 | DV自动检测启用 | `decode check dv` 日志出现 | `dmesg | grep "decode check dv"` |
| 2 | Slave Decoder创建 | `created slave decoder` 日志出现 | `dmesg | grep "slave decoder"` |
| 3 | BL/EL切换正常 | `switch (poc XX) to el` 日志出现 | `dmesg | grep "switch.*el"` |
| 4 | 播放测试 | 画面正常显示 | 播放DV FEL文件 |
| 5 | 快退快进 | 无乱码、花屏 | 执行快退快进操作 |
| 6 | 无错误日志 | 无 `bl not found el` | `dmesg | grep "not found el"` |

### 4.3 重点关注日志

```bash
# 实时查看关键日志
dmesg -w | grep -E "DV|h265|slave|el|bl|FEL_DBG"
```

**关注日志**:
- ✅ `decode check dv` - DV自动检测启用
- ✅ `created slave decoder` - Slave解码器创建成功
- ✅ `[FEL_DBG] S5: el_enable=1` - EL解码启用
- ✅ `switch (poc XX) to el` - BL/EL切换正常
- ❌ `bl not found el` - 错误：BL找不到EL
- ❌ `cur lcu idx = 0, set error_mark` - EL解码失败

---

## 五、问题排查指南

### 5.1 常见问题分析

| 问题现象 | 可能原因 | 排查方向 |
|---------|---------|---------|
| 无 `decode check dv` | `check_dv_flag` 未设置 | 检查配置读取代码 |
| 无 `slave decoder` | debugflags 未设置或条件不满足 | 检查 `port->type & PORT_TYPE_DUALDEC` |
| LCU错误 | EL解码器数据错误 | 检查解码模式配置 |
| `bl not found el` | BL/EL帧配对失败 | 检查 POC 同步机制 |

### 5.2 调试命令参考

```bash
# 检查调试标志
cat /sys/module/decoder_common/parameters/debugflags

# 检查解码器设备
ls -la /dev/vpu /dev/hevc*

# 检查DV相关模块参数
ls /sys/module/aml_media/parameters/ | grep dv

# 查看详细日志
dmesg | grep -E "dv_duallayer|check_dv_flag|is_dv_flag"
```

---

## 六、待完成工作

| 任务 | 状态 | 负责人 | 说明 |
|------|------|--------|------|
| 内核重新编译 | ⏳ 待执行 | 后续人员 | 需要在CoreELEC环境编译 |
| 固件部署 | ⏳ 待执行 | 后续人员 | 部署到S5设备 |
| 功能测试 | ⏳ 待执行 | 后续人员 | 播放DV FEL文件验证 |
| 快退快进测试 | ⏳ 待执行 | 后续人员 | 验证禁用front_back_mode后的功能 |
| 性能优化 | 🔄 进行中 | 后续人员 | 根据测试结果调整 |

---

## 七、架构说明

### 7.1 双核DV解码架构

```
┌──────────────────────────────────────────────┐
│              h265_fb 解码器                   │
│                                              │
│  ┌─────────────────┐  ┌─────────────────┐  │
│  │  BL解码器        │  │  EL解码器        │  │
│  │  vdec->master   │◄─┼─►│  vdec->slave   │  │
│  │  (硬件解码)      │  │  (硬件解码)      │  │
│  └─────────────────┘  └─────────────────┘  │
│           │                    │             │
│           │    POC 同步        │             │
│           └─────────┬──────────┘             │
│                     │                        │
│                     ▼                        │
│  ┌──────────────────────────────────────┐   │
│  │  vframe (aux_data = RPU)              │   │
│  │  dv_enhance_exist = 1                │   │
│  └──────────────────────────────────────┘   │
│                    │                        │
│                    ▼                        │
│  ┌──────────────────────────────────────┐   │
│  │  amdv 模块（内核）                     │   │
│  │  • 解析RPU                            │   │
│  │  • 应用映射曲线                        │   │
│  │  • EL残差应用                         │   │
│  │  • 最终帧合成                         │   │
│  └──────────────────────────────────────┘   │
└──────────────────────────────────────────────┘
```

### 7.2 职责边界

| 模块 | 职责 |
|------|------|
| **h265_fb 解码器** | 硬件解码BL/EL帧、提取RPU元数据、POC同步 |
| **amdv 模块** | RPU解析、映射曲线应用、残差处理、帧合成 |

---

## 八、联系方式

如有问题，可联系原开发者获取进一步支持。

---

**文档版本**: v1.0  
**创建日期**: 2026-05-31  
**适用分支**: `glsimon/20260530`  
**作者**: glsimon