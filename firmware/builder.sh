#!/bin/bash
#
# OpenIPC | version 2023.11.30

# Autoupdate BUILDER repo
# Remove old building folder (for full rebuild)
# Download OpenIPC repo
# Copy files from Device Overlay
# Build Firmware
# Copy Kernel and Rootfs to Archive

DEVICE="$1"
BUILDER_DIR=$(pwd)
FIRMWARE_DIR="${BUILDER_DIR}/openipc"
TIMESTAMP=$(date +"%Y%m%d%H%M")
VERSION=$(stat -c"%Y" $0)

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

copy_to_archive() {
    echo_c 32 "Copying files to local archive"
    mkdir -p "${BUILDER_DIR}/archive/${DEVICE}/${TIMESTAMP}"
    cp -a \
        ${FIRMWARE_DIR}/output/images/rootfs.squashfs.* \
        ${FIRMWARE_DIR}/output/images/uImage.* \
        ${FIRMWARE_DIR}/output/images/*.tar \
        ${FIRMWARE_DIR}/output/images/openipc.*.tgz \
        ${BUILDER_DIR}/archive/${DEVICE}/${TIMESTAMP}

    if ls ${FIRMWARE_DIR}/output/images/u-boot-*-nor.bin >/dev/null 2>&1; then
        cp -a ${FIRMWARE_DIR}/output/images/u-boot-*.bin ${BUILDER_DIR}/archive/${DEVICE}/${TIMESTAMP}
    fi

    echo_c 35 "\nAssembled firmware available in:"
    tree -C "${BUILDER_DIR}/archive/${DEVICE}/${TIMESTAMP}"
}

select_device() {
    AVAILABLE_DEVICES=$(find devices -name *_defconfig | sort | cut -d/ -f5)
    cmd="whiptail --title \"Available devices\" --menu \"Please select a device from the list below:\" 20 70 12"
    for p in ${AVAILABLE_DEVICES//_defconfig}; do cmd="${cmd} \"$p\" \"\""; done
    DEVICE=$(eval "${cmd} 3>&1 1>&2 2>&3")
    if [ $? != 0 ]; then
        echo_c 31 "Cancelled."
        exit 1
    fi
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

echo_c 37 "Experimental system for building OpenIPC firmware for known devices"
echo_c 30 "https://openipc.org/"
echo_c 30 "Version: ${VERSION}"

while [ -z "${DEVICE}" ]; do select_device; done

echo_c 31 "\nStarting a device for ${DEVICE}"
ITEM=$(find devices -name ${DEVICE}_defconfig | cut -d/ -f1,2)
tree -C "${ITEM}"

sleep 3

echo_c 33 "\nUpdating Builder"
git pull

rm -rf openipc
if [ ! -d "$FIRMWARE_DIR" ]; then
    echo_c 33 "\nDownloading Firmware"
    git clone --depth=1 https://github.com/OpenIPC/firmware.git "$FIRMWARE_DIR"
    cd "$FIRMWARE_DIR"
else
    echo_c 33 "\nUpdating Firmware"
    cd "$FIRMWARE_DIR"
    # git reset HEAD --hard
    # git pull --rebase
fi

echo_c 33 "\nCopying extra packages"
copy_extra_packages

echo_c 33 "\nCopying device files"
# Every board here is a mabur drone on the same SSC338Q boot chain, so the
# shared overlay (rcS, load_sigmastar, sensor bins, ...) lives once in
# devices/_mabur-common and the board dir, copied on top, holds only what
# differs (defconfig, customizer.sh). _mabur-common has no *_defconfig, so
# select_device never offers it as a board.
cp -afv ${BUILDER_DIR}/devices/_mabur-common/* ${FIRMWARE_DIR}
cp -afv ${BUILDER_DIR}/${ITEM}/* ${FIRMWARE_DIR}

echo_c 33 "\nBuilding the device"
make BOARD=${DEVICE}

build_uboot

copy_to_archive
echo_c 35 "\nDone"
cd "$BUILDER_DIR"
