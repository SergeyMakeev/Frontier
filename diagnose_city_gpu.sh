#!/usr/bin/env bash
# Read-only Linux GPU diagnostics. Run from the SBC's local desktop terminal.
# No packages, drivers, permissions, or persistent environment settings change.
set -uo pipefail

if [[ "$(uname -s)" != Linux ]]; then
    echo "Run this script on the Linux SBC, in the same desktop session as the city sample." >&2
    exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${FRONTIER_CITY_BUILD_DIR:-${ROOT_DIR}/build-city}"

section() { printf '\n=== %s ===\n' "$1"; }

probe() {
    local executable="$1" status=0
    shift
    if ! command -v "${executable}" >/dev/null 2>&1; then
        printf '%s is not installed; skipping.\n' "${executable}"
        return
    fi
    printf '+ %s' "${executable}"
    if (( $# != 0 )); then printf ' %q' "$@"; fi
    printf '\n'
    if command -v timeout >/dev/null 2>&1; then
        timeout --kill-after=2s 20s "${executable}" "$@" 2>&1 || status=$?
    else
        printf 'timeout is unavailable; skipping the graphics probe to avoid hanging.\n'
        return
    fi
    if (( status != 0 )); then
        printf '%s exited with status %d (124 means timeout).\n' "${executable}" "${status}"
    fi
}

section "Board and OS"
uname -srmo
for path in /proc/device-tree/model /proc/device-tree/compatible; do
    if [[ -r "${path}" ]]; then
        printf '%s:\n' "${path}"
        tr '\0' '\n' < "${path}"
        printf '\n'
    fi
done
for path in /etc/os-release /etc/armbian-release; do
    if [[ -r "${path}" ]]; then
        printf '%s:\n' "${path}"
        # Select useful identifiers; do not source system files as shell code.
        awk '/^(PRETTY_NAME|ID|VERSION_ID|BOARD|BOARD_NAME|BOARDFAMILY|BRANCH|VERSION|LINUXFAMILY)=/' "${path}"
    fi
done

section "Graphics environment (only graphics-related variables)"
for name in XDG_SESSION_TYPE XDG_CURRENT_DESKTOP DISPLAY WAYLAND_DISPLAY SDL_VIDEODRIVER \
    SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR SDL_VIDEO_WAYLAND_PREFER_LIBDECOR EGL_PLATFORM EGL_LOG_LEVEL \
    LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT LIBGL_DRIVERS_PATH \
    MESA_LOADER_DRIVER_OVERRIDE MESA_GL_VERSION_OVERRIDE MESA_GLES_VERSION_OVERRIDE \
    GALLIUM_DRIVER DRI_PRIME MESA_VK_DEVICE_SELECT VK_DRIVER_FILES VK_ICD_FILENAMES \
    __EGL_VENDOR_LIBRARY_FILENAMES __EGL_VENDOR_LIBRARY_DIRS LD_LIBRARY_PATH LD_PRELOAD; do
    if [[ -v "${name}" ]]; then
        printf '%s=%s\n' "${name}" "${!name}"
    fi
done

section "GPU devices and current-user access"
printf 'Group memberships: '
id -Gn
shopt -s nullglob
devices=(/dev/dri/card* /dev/dri/renderD* /dev/mali*)
if (( ${#devices[@]} == 0 )); then
    echo "No /dev/dri/card*, /dev/dri/renderD*, or /dev/mali* devices found."
fi
for path in "${devices[@]}"; do
    ls -l "${path}"
    if [[ -r "${path}" && -w "${path}" ]]; then
        printf '  Current user has read/write access.\n'
    else
        printf '  Current user lacks read or write access.\n'
    fi
done
for path in /sys/class/drm/card*/device/driver /sys/class/drm/renderD*/device/driver; do
    printf '%s -> %s\n' "${path}" "$(readlink -f "${path}")"
done
if [[ -r /proc/modules ]]; then
    echo "Relevant loaded modules (built-in drivers will not appear):"
    awk 'tolower($1) ~ /panfrost|panthor|mali|rockchip|drm/ { print }' /proc/modules
fi

section "Installed graphics packages"
if command -v dpkg-query >/dev/null 2>&1; then
    dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Abbrev}\n' \
        libegl1 libegl-mesa0 libgl1-mesa-dri libglx-mesa0 mesa-vulkan-drivers \
        mesa-utils mesa-utils-extra 'libmali*' libsdl2-2.0-0 \
        libdecor-0-0 libdecor-0-plugin-1-gtk libdecor-0-plugin-1-cairo 2>&1 || true
fi

section "City build configuration"
if [[ -r "${BUILD_DIR}/CMakeCache.txt" ]]; then
    awk '/^(BGFX_(OPENGL_VERSION|OPENGLES_VERSION|CONFIG_RENDERER_[A-Z0-9_]+|WITH_WAYLAND)|FRONTIER_CITY_[A-Z0-9_]+):/ { print }' \
        "${BUILD_DIR}/CMakeCache.txt"
else
    printf 'No CMake cache at %s\n' "${BUILD_DIR}/CMakeCache.txt"
fi

section "Desktop OpenGL / GLX"
LIBGL_DEBUG=verbose probe glxinfo -B

section "EGL (the city sample uses this path on Linux)"
# EGL may report several platforms/devices. Keep their headings: hardware on
# a surfaceless device does not establish acceleration for the X11 window.
EGL_LOG_LEVEL=debug LIBGL_DEBUG=verbose probe eglinfo -B

section "OpenGL ES / EGL on X11"
# Also useful with older eglinfo versions that list configurations without
# printing a renderer for each client API. This probe uses an X11 window.
EGL_LOG_LEVEL=debug LIBGL_DEBUG=verbose probe es2_info

section "Vulkan"
probe vulkaninfo --summary

section "GPU kernel messages, if readable without root"
if command -v dmesg >/dev/null 2>&1; then
    dmesg --color=never | awk 'tolower($0) ~ /panfrost|panthor|mali|gpu|drm/ { print }' || true
fi

section "Interpretation"
cat <<'TEXT'
llvmpipe, softpipe, and lavapipe render on the CPU. "Direct rendering: Yes"
alone does not establish hardware acceleration.

Compare the renderer for the active window platform, not just a surfaceless
EGL device. The city requests desktop OpenGL through EGL on Linux, with SDL2
selecting native Wayland in a Wayland desktop unless SDL_VIDEODRIVER is set.
If Wayland uses Mali/Panfrost but X11 uses llvmpipe with DRI3 errors, run:
  SDL_VIDEODRIVER=wayland bash ./run_city_sample.sh --gl
Verify "Window system: Wayland" and the hardware GPU name in Frontier debug.

The city draws a Wayland title bar and resize border by default. Drag the title
to move, or an edge/corner to resize. Plain --gl uses 4x MSAA on Wayland, which
removed triangle seams on the reported Mali-G52/Panfrost system.

The optional --native-window-frame path may need SDL's libdecor runtime/plugin:
  sudo apt-get install libdecor-0-0 libdecor-0-plugin-1-gtk
The city also supports Alt + left drag to move and Alt + Shift + left drag to
resize, even when no decoration provider is available.

Its default bgfx build does not enable OpenGL ES; a runtime --gles flag alone
cannot enable a renderer that was not compiled in.

If GL and EGL both use software, inspect driver loading, device access, and
the graphics environment above. If EGL OpenGL ES uses Mali/Panfrost while
desktop OpenGL uses software, an ES build may resolve the API mismatch.
Vulkan can also use software, so check its device name/type before comparing FPS.

For the application's own EGL initialization log, run from the desktop:
  EGL_LOG_LEVEL=debug LIBGL_DEBUG=verbose bash ./run_city_sample.sh --gl --no-msaa 2>&1 | tee city-egl.log
Close the sample after it starts, then share this report and city-egl.log.
TEXT
