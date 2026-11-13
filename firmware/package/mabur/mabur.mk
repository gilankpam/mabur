################################################################################
#
# mabur
#
################################################################################

# Tracks master HEAD rather than a fixed SHA. Same reasoning and same
# mechanics as devourer.mk: buildroot's git backend cannot check out a branch
# literally named "master" (it refuses to fetch into the branch `git init`
# already has checked out), so resolve it to a commit id here.
MABUR_SITE = https://github.com/gilankpam/mabur
MABUR_BRANCH = master
# A MABUR_VERSION already in the environment wins: the release workflow
# resolves the commit once and passes it in, so the release it names is
# the commit that was built even if the branch moves mid-build.
ifeq ($(MABUR_VERSION),)
MABUR_VERSION := $(shell git ls-remote $(MABUR_SITE) refs/heads/$(MABUR_BRANCH) | cut -f1)
endif
ifeq ($(MABUR_VERSION),)
$(error mabur: cannot resolve $(MABUR_BRANCH) at $(MABUR_SITE) (no network?))
endif
MABUR_SITE_METHOD = git
MABUR_LICENSE = MIT
MABUR_SUPPORTS_IN_SOURCE_BUILD = NO

# devourer comes from its own package, NOT from mabur's git submodule
# (MABUR_GIT_SUBMODULES is deliberately unset). The submodule records a fixed
# SHA that lags the fork's master, and the in-tree cross-build
# (tools/build-arm.sh) does not use the submodule either -- it points
# DEVOURER_DIR at a sibling checkout. Sourcing devourer separately keeps the
# buildroot build consistent with that and lets both trees float.
MABUR_DEPENDENCIES = libusb host-pkgconf devourer

# BUILD_SHARED_LIBS=OFF: devourer's add_library(devourer ...) has no explicit
# STATIC/SHARED, so it follows BUILD_SHARED_LIBS. Buildroot defaults that ON,
# which built libdevourer.so -- but the recipe installs only maburd, so on target
# maburd died with "error while loading shared libraries: libdevourer.so".
# Forcing it OFF links devourer (and mabur_common) statically into maburd, the
# same self-contained layout tools/build-arm.sh produces; the only remaining
# runtime deps are Buildroot's own libusb/libstdc++/libc, which are on the image.
# (maburd also dlopens the SigmaStar MI libraries from the vendor rootfs at
# runtime -- that is why it must stay a glibc DYNAMIC executable.)
#
# DEVOURER_LOG_MAX_LEVEL=WARN: compile out info/debug/trace. devourer logs one
# info line per TX frame ("bulk_send EP 5 OK N bytes"); at maburd's frame rate
# that floods RAM-backed /tmp/mabur.log until the next respawn truncates it.
#
# The DEVOURER_* chip selects mirror tools/build-arm.sh. Without them every
# Realtek family is compiled in -- including DEVOURER_8733B, which defaults ON
# upstream and must be OFF here. Keep the two lists in step. The one
# deliberate difference: the Jaguar3 die is the board's Kconfig radio choice
# (Config.in), where build-arm.sh is always the 8822E -- the bench drone's
# card. Exactly one of the two is ON, and maburd relies on that: it picks
# devourer's wide-TX IGI floor from the die compiled in (none on an
# 8822E-only build, devourer's default 0x35 otherwise). With both ON an
# 8812EU would fly the CU's floor and lose uplink sensitivity at range.
#
# MABUR_BUILD_GS/LINKBENCH=OFF: this is the drone image. maburgs and the bench
# harnesses are neither installed nor useful on the SSC338Q.
MABUR_CONF_OPTS = \
	-DDEVOURER_DIR=$(DEVOURER_DIR) \
	-DMABUR_BUILD_TESTS=OFF \
	-DMABUR_BUILD_DRONE=ON \
	-DMABUR_BUILD_GS=OFF \
	-DMABUR_BUILD_LINKBENCH=OFF \
	-DBUILD_SHARED_LIBS=OFF \
	-DDEVOURER_LOG_MAX_LEVEL=WARN \
	-DDEVOURER_JAGUAR1=OFF \
	-DDEVOURER_8814=OFF \
	-DDEVOURER_JAGUAR2_8822B=OFF \
	-DDEVOURER_JAGUAR2_8821C=OFF \
	-DDEVOURER_8733B=OFF \
	-DDEVOURER_KESTREL_8852B=OFF \
	-DDEVOURER_KESTREL_8852C=OFF

ifeq ($(BR2_PACKAGE_MABUR_RADIO_8812CU),y)
MABUR_CONF_OPTS += -DDEVOURER_JAGUAR3_8822C=ON -DDEVOURER_JAGUAR3_8822E=OFF
else
MABUR_CONF_OPTS += -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=ON
endif

# Config, init script and binary all come from the same floating master
# checkout, which is the whole point: an unknown key fails boot and the wrapper
# then respawns maburd forever at 2 s, so the three must never be mixed across
# commits. The json/toml either-or that used to live here is gone -- the TOML
# cutover landed on master and bundle/mabur.default.json no longer exists.
#
# Since 2026-09-08 that bundle file is not a neutral seed but a verbatim copy
# of the drone's own /etc/mabur.toml, so a fresh flash boots the flight
# configuration rather than something nobody has flown. Retune it in the mabur
# repo (tests/test_config.cpp pins it), not only on the device.
define MABUR_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(MABUR_BUILDDIR)/drone/maburd $(TARGET_DIR)/usr/bin/maburd
	# Installed as S00mabur, not S96mabur: busybox rcS runs /etc/init.d/S*
	# strictly serially in lexical order, and maburd goes FIRST -- ahead of
	# seedrng, syslogd, fake-hwclock, sysctl, customizer, mdev, network, ntpd,
	# dropbear and crond, none of which video needs (devtmpfs makes the
	# device nodes; mdev only adds SD-card automount rules). The MI modules
	# are no longer loaded by an init script at all: maburd runs
	# load_sigmastar itself (a constant in drone/src/main.cpp) under its USB port reset,
	# so the insmod chain lands under the radio bring-up instead of ahead of
	# maburd's exec. Measured 2026-09-09: first AU on the GS 5.40 -> 4.37 s
	# of uptime (mabur docs/boot-time-findings-2026-09-07.md, "rcS, stamped").
	$(INSTALL) -D -m 0755 $(@D)/bundle/S96mabur $(TARGET_DIR)/etc/init.d/S00mabur
	$(INSTALL) -D -m 0644 $(@D)/bundle/mabur.default.toml $(TARGET_DIR)/etc/mabur.toml
endef

$(eval $(cmake-package))
