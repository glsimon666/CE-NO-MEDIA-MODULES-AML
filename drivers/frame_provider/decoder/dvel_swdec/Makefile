MODULE_NAME = amvdec_dvel
obj-$(CONFIG_AMLOGIC_MEDIA_VDEC_DVEL) += ${MODULE_NAME}.o
${MODULE_NAME}-objs += dvel_swdec.o

# Enable building when CONFIG_AMLOGIC_MEDIA_VDEC_H265_FB is set
ifneq ($(CONFIG_AMLOGIC_MEDIA_VDEC_H265_FB),)
obj-m += ${MODULE_NAME}.o
endif

PR_FMT = $(subst amvdec_,,$(MODULE_NAME))
PR_FMT_DEFINE = "-Dpr_fmt(fmt)=\"[$(PR_FMT)]:\" fmt"
ccflags-y += $(PR_FMT_DEFINE)
