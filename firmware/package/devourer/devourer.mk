################################################################################
#
# devourer
#
################################################################################

# The commit mabur's third_party/devourer pins; firmware/builder.sh reads it
# and passes it in the environment, so the drone and the GS (whose mabur pin
# carries the same gitlink) build the same devourer. Buildroot gets a plain
# SHA: its git backend cannot fetch a branch name that matches the HEAD its
# own `git init` creates, and dl/ is keyed on the version string.
DEVOURER_SITE = https://github.com/gilankpam/devourer
ifeq ($(DEVOURER_VERSION),)
$(error devourer: DEVOURER_VERSION is not set (build through firmware/builder.sh))
endif
DEVOURER_SITE_METHOD = git
DEVOURER_LICENSE = GPL-2.0
DEVOURER_LICENSE_FILES = LICENSE

# Source-only. No configure/build/install steps are defined on purpose: the
# tree is consumed by mabur's CMakeLists (add_subdirectory), never built or
# installed standalone. Chip-family selection and the log-level floor are
# passed by mabur.mk, since they are compile-time options of the resulting
# maburd, not of anything this package produces.
DEVOURER_INSTALL_TARGET = NO
DEVOURER_INSTALL_STAGING = NO

$(eval $(generic-package))
