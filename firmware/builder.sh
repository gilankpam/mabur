#!/usr/bin/env bash
#
# Builds the mabur drone firmware for one board:
#   firmware/builder.sh <board>        e.g. ssc338q_fpv_openipc-urllc-aio
#
# Fetches OpenIPC/firmware at the commit in firmware/openipc.ref, copies this
# directory's packages and device overlay into it, builds the image (maburd
# from the mabur tree this script sits in, not a fetched master), builds
# U-Boot from our fork, and writes output/images/build-info.txt naming what
# went in. Non-interactive; any failure exits non-zero.
#
# Environment:
#   FIRMWARE_DIR      the OpenIPC tree (default: firmware/openipc)
#   MABUR_SRC_DIR     the mabur tree maburd is built from (default: repo root)
#   DEVOURER_VERSION  devourer commit (default: devourer master, resolved once)
#   SKIP_UBOOT=1      leave U-Boot alone
set -eo pipefail

DEVICE="$1"
BUILDER_DIR=$(cd "$(dirname "$0")" && pwd)
FIRMWARE_DIR="${FIRMWARE_DIR:-${BUILDER_DIR}/openipc}"
MABUR_SRC_DIR="${MABUR_SRC_DIR:-$(cd "${BUILDER_DIR}/.." && pwd)}"
export MABUR_SRC_DIR
OPENIPC_REPO="https://github.com/OpenIPC/firmware.git"
OPENIPC_REF=$(tr -d '[:space:]' < "${BUILDER_DIR}/openipc.ref")

# U-Boot is built from our fork rather than pulled prebuilt from OpenIPC
# releases: the fork carries the boot-time fixes for the mabur FPV link
# (CONFIG_ETHERNET_FIXLINK, which removes a 4 s auto-negotiation timeout when
# no Ethernet is attached, plus the SD boot-script probe guard).  Set
# SKIP_UBOOT=1 to leave U-Boot alone.
UBOOT_REPO="${UBOOT_REPO:-https://github.com/gilankpam/u-boot-sigmastar.git}"
UBOOT_REF="${UBOOT_REF:-mabur-fastboot}"
UBOOT_DIR="${UBOOT_DIR:-${BUILDER_DIR}/u-boot-sigmastar}"
# NOR "boot" partition size, i.e. how far the image is padded so no stale
# tail survives a flashcp.  256k on every ssc338q board seen so far.
UBOOT_PART_SIZE="${UBOOT_PART_SIZE:-262144}"

echo_c() {
    # 30 grey, 31 red, 32 green, 33 yellow, 34 blue, 35 magenta, 36 cyan, 37 white
    t="\e[1;$1m$2\e[0m" || t="$2"
    echo -e "$t"
}

uboot_family() {
    # Mirrors the SoC -> defconfig grouping in the U-Boot tree's build.sh.
    case "$1" in
        ssc325|ssc325de)                        echo infinity6 ;;
        ssc333|ssc335|ssc337|ssc335de|ssc337de) echo infinity6b0 ;;
        ssc377*|ssc378*)                        echo infinity6c ;;
        ssc30kd|ssc30kq|ssc338q)                echo infinity6e ;;
        *)                                      echo "" ;;
    esac
}

build_uboot() {
    [ -n "${SKIP_UBOOT}" ] && { echo_c 33 "\nSKIP_UBOOT set, not building U-Boot"; return 0; }

    local soc family cross out
    soc=$(echo ${DEVICE} | cut -d_ -f1)
    family=$(uboot_family "${soc}")
    if [ -z "${family}" ]; then
        echo_c 33 "\nNo U-Boot family mapping for ${soc}, skipping U-Boot"
        return 0
    fi

    # Reuse the Buildroot toolchain that has just been built, so U-Boot needs
    # no cross compiler of its own.  It only exists after the device build.
    cross="${FIRMWARE_DIR}/output/host/bin/arm-openipc-linux-gnueabihf-"
    if [ ! -x "${cross}gcc" ]; then
        echo_c 31 "\nBuildroot toolchain missing, skipping U-Boot"
        return 0
    fi

    echo_c 34 "\nFetching U-Boot (${UBOOT_REPO} @ ${UBOOT_REF})"
    if [ ! -d "${UBOOT_DIR}/.git" ]; then
        git clone "${UBOOT_REPO}" "${UBOOT_DIR}" || return 1
    fi
    ( cd "${UBOOT_DIR}" && git fetch origin "${UBOOT_REF}" && git checkout -q FETCH_HEAD ) || return 1

    echo_c 34 "\nBuilding U-Boot for ${soc} (${family})"
    (
        cd "${UBOOT_DIR}" || exit 1
        export ARCH=arm CROSS_COMPILE="${cross}"
        make distclean >/dev/null 2>&1
        make ${family}_defconfig || exit 1
        make -j"$(nproc)" KCFLAGS=-DPRODUCT_SOC=${soc} || exit 1
        sh make_boot_spinor.sh ${family} || exit 1
    ) || { echo_c 31 "\nU-Boot build FAILED"; return 1; }

    out="${FIRMWARE_DIR}/output/images"
    mkdir -p "${out}"
    cp "${UBOOT_DIR}/BOOT.bin" "${out}/u-boot-${soc}-nor.bin"
    # Padded to the whole boot partition so a flashcp leaves no stale tail.
    # Pad with 0xFF, the erased state of NOR -- truncate would pad with 0x00,
    # which programs bits for no reason.  Same idiom as make_boot_spinor.sh.
    dd if=/dev/zero bs=1k count=$((UBOOT_PART_SIZE / 1024)) status=none \
        | tr '\000' '\377' > "${out}/u-boot-${soc}-nor-padded.bin"
    dd if="${UBOOT_DIR}/BOOT.bin" of="${out}/u-boot-${soc}-nor-padded.bin" \
        conv=notrunc status=none

    echo_c 32 "\nU-Boot built: $(ls -l ${out}/u-boot-${soc}-nor.bin | awk '{print $5}') bytes"
    echo_c 33 "Flash with:  flashcp u-boot-${soc}-nor-padded.bin /dev/mtd0"
    echo_c 33 "(NOT the ubnor env command -- it erases 0x0..0x50000 and takes"
    echo_c 33 " the U-Boot environment, including ethaddr, with it.)"
}

copy_extra_packages() {
    extra_package=${BUILDER_DIR}/package
    firmware_package=${FIRMWARE_DIR}/general/package
    cp -afv $extra_package/* $firmware_package
    package_list_file=$firmware_package/Config.in
    for f in "$extra_package"/*
    do
        package_name=$(basename $f)
        if ! grep -Fq "$package_name" $package_list_file
        then
            printf 'source "$BR2_EXTERNAL_GENERAL_PATH/package/%s/Config.in"\n' $package_name >> $package_list_file
        fi
    done
}

fetch_openipc() {
    if [ "$(git -C "${FIRMWARE_DIR}" rev-parse HEAD 2>/dev/null)" = "${OPENIPC_REF}" ]; then
        echo_c 33 "\nOpenIPC firmware tree already at ${OPENIPC_REF}"
        return 0
    fi
    echo_c 33 "\nFetching OpenIPC firmware ${OPENIPC_REF}"
    mkdir -p "${FIRMWARE_DIR}"
    git -C "${FIRMWARE_DIR}" init -q
    git -C "${FIRMWARE_DIR}" fetch -q --depth 1 "${OPENIPC_REPO}" "${OPENIPC_REF}"
    git -C "${FIRMWARE_DIR}" checkout -q --force FETCH_HEAD
}

sha256_of() {
    # The first file matching the glob, or "none" when the build did not
    # download one.
    local f
    for f in $1; do
        [ -f "$f" ] && { sha256sum "$f" | cut -d' ' -f1; return; }
    done
    echo none
}

write_build_info() {
    local dl out
    dl=$(ls -d "${FIRMWARE_DIR}"/output/buildroot-*/dl | head -1)
    out="${FIRMWARE_DIR}/output/images/build-info.txt"
    {
        echo "board=${DEVICE}"
        echo "openipc=${OPENIPC_REF}"
        echo "kernel=${KERNEL_REF}"
        echo "devourer=${DEVOURER_VERSION}"
        echo "uboot=$(git -C "${UBOOT_DIR}" rev-parse HEAD 2>/dev/null || echo none)"
        echo "kernel_sha256=$(sha256_of "${dl}/linux/${KERNEL_REF}.tar.gz")"
        echo "toolchain_sha256=$(sha256_of "${dl}/toolchain-external-custom/*.tgz")"
        echo "ipctool_sha256=$(sha256_of "${dl}/ipctool/*.tar.gz")"
    } > "${out}"
    cat "${out}"
}

if [ -z "${DEVICE}" ]; then
    echo "usage: $0 <board>   (one of: $(cd "${BUILDER_DIR}/devices" && ls -d ssc*_fpv_* | tr '\n' ' '))" >&2
    exit 2
fi
ITEM=$(cd "${BUILDER_DIR}" && find devices -name "${DEVICE}_defconfig" | cut -d/ -f1,2)
if [ -z "${ITEM}" ]; then
    echo "error: no board ${DEVICE} under ${BUILDER_DIR}/devices" >&2
    exit 2
fi
if [ -z "${DEVOURER_VERSION}" ]; then
    DEVOURER_VERSION=$(git ls-remote https://github.com/gilankpam/devourer refs/heads/master | cut -f1)
fi
[ -n "${DEVOURER_VERSION}" ] || { echo "error: cannot resolve devourer master" >&2; exit 1; }
export DEVOURER_VERSION
# The kernel commit the board's defconfig pins (its tarball URL ends in it).
KERNEL_REF=$(sed -n 's|^BR2_LINUX_KERNEL_CUSTOM_TARBALL_LOCATION=".*/archive/\([0-9a-f]\{40\}\)\.tar\.gz"$|\1|p' \
    "${BUILDER_DIR}/${ITEM}/br-ext-chip-sigmastar/configs/${DEVICE}_defconfig")
[ -n "${KERNEL_REF}" ] || { echo "error: ${DEVICE}_defconfig does not pin a kernel commit" >&2; exit 1; }

echo_c 31 "\nBuilding ${DEVICE} (mabur from ${MABUR_SRC_DIR})"
fetch_openipc
cd "${FIRMWARE_DIR}"

echo_c 33 "\nCopying extra packages"
copy_extra_packages
echo_c 33 "\nCopying device files"
# Every board here is a mabur drone on the same SSC338Q boot chain, so the
# shared overlay (rcS, load_sigmastar, sensor bins, ...) lives once in
# devices/_mabur-common and the board dir, copied on top, holds only what
# differs (defconfig, customizer.sh).
cp -af "${BUILDER_DIR}"/devices/_mabur-common/* "${FIRMWARE_DIR}"
cp -af "${BUILDER_DIR}/${ITEM}"/* "${FIRMWARE_DIR}"

echo_c 33 "\nBuilding the device"
make BOARD="${DEVICE}"
build_uboot
write_build_info
echo_c 35 "\nDone: ${FIRMWARE_DIR}/output/images"
