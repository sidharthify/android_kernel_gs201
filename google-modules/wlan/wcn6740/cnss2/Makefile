KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

# $(WLAN_PLATFORM_ROOT) has to be a absolute path
ifeq ($(WLAN_PLATFORM_ROOT),)
WLAN_PLATFORM_ROOT = $(srctree)/$(src)

# If it reaches here, compilation is probably without Android.mk,
# so enable all flags (including debug flag CONFIG_CNSS2_DEBUG) by
# default.
KBUILD_OPTIONS := WLAN_PLATFORM_ROOT=$(WLAN_PLATFORM_ROOT)
KBUILD_OPTIONS += CONFIG_CNSS_OUT_OF_TREE=y
KBUILD_OPTIONS += CONFIG_CNSS2=m
KBUILD_OPTIONS += CONFIG_CNSS2_QMI=y
KBUILD_OPTIONS += CONFIG_CNSS2_DEBUG=y
KBUILD_OPTIONS += CONFIG_CNSS_QMI_SVC=m
KBUILD_OPTIONS += CONFIG_CNSS_PLAT_IPC_QMI_SVC=m
KBUILD_OPTIONS += CONFIG_CNSS_GENL=m
KBUILD_OPTIONS += CONFIG_WCNSS_MEM_PRE_ALLOC=m
KBUILD_OPTIONS += CONFIG_CNSS_UTILS=m

# MHI
KBUILD_OPTIONS += CONFIG_MHI_BUS=m
KBUILD_OPTIONS += CONFIG_MHI_BUS_MISC=y
KBUILD_OPTIONS += CONFIG_MHI_BUS_DEBUG=y

# QMI
KBUILD_OPTIONS += CONFIG_QCOM_QMI_HELPERS=m

# QRTR
KBUILD_OPTIONS += CONFIG_QRTR=m
KBUILD_OPTIONS += CONFIG_QRTR_MHI=m
KBUILD_OPTIONS += CONFIG_QRTR_NODE_ID=1
KBUILD_OPTIONS += CONFIG_QRTR_WAKEUP_MS=0

# WLAN_MAC
KBUILD_OPTIONS += CONFIG_GOOGLE_WLAN_MAC=m

EXTRA_SYMBOLS +=$(OUT_DIR)/../private/google-modules/soc/gs/Module.symvers

endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)"

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
