#!/bin/bash

# Hardware Encoder Detection Script (No FFmpeg, No Compilation Required)
# Copyright 2024 coScene
# Usage: ./hw_encoder_detect.sh

echo "Hardware Encoder Detection Script (Pure Bash)"
echo "=============================================="
echo

# Function to check if file exists
file_exists() {
    [ -f "$1" ] || [ -c "$1" ] || [ -b "$1" ]
}

# Function to check if library can be loaded
can_load_lib() {
    ldconfig -p | grep -q "$1" 2>/dev/null || \
    [ -f "/usr/lib/x86_64-linux-gnu/$1" ] || \
    [ -f "/usr/lib/$1" ] || \
    [ -f "/lib/x86_64-linux-gnu/$1" ] || \
    [ -f "/lib/$1" ]
}

# Function to read first line of file
read_first_line() {
    if [ -r "$1" ]; then
        head -n1 "$1" 2>/dev/null
    fi
}

# NVIDIA NVENC Detection
detect_nvenc() {
    echo "Encoder: h264_nvenc"
    echo "Type: NVIDIA NVENC"
    
    local confidence=0
    local reasons=""
    local device=""
    
    # Check NVIDIA device
    if file_exists "/dev/nvidia0"; then
        device="/dev/nvidia0"
        confidence=$((confidence + 50))
        reasons="${reasons}NVIDIA device found; "
    else
        reasons="${reasons}NVIDIA device not found; "
    fi
    
    # Check NVIDIA driver
    if [ -r "/proc/driver/nvidia/version" ]; then
        local driver_info=$(read_first_line "/proc/driver/nvidia/version")
        if [ -n "$driver_info" ]; then
            confidence=$((confidence + 20))
            reasons="${reasons}NVIDIA driver loaded; "
        else
            reasons="${reasons}NVIDIA driver not found; "
        fi
    fi
    
    # Check NVENC library
    if can_load_lib "libnvidia-encode.so" || can_load_lib "libnvidia-encode.so.1"; then
        confidence=$((confidence + 30))
        reasons="${reasons}NVENC library available; "
    else
        reasons="${reasons}NVENC library NOT available; "
    fi

    local available="NO"
    if [ $confidence -ge 50 ]; then
        available="YES"
    fi
    
    echo "Available: $available"
    echo "Confidence: ${confidence}%"
    [ -n "$device" ] && echo "Device: $device"
    echo "Details: $reasons"
    echo
}

# Intel Quick Sync Detection
detect_qsv() {
    echo "Encoder: h264_qsv"
    echo "Type: Intel Quick Sync"
    
    local confidence=0
    local reasons=""
    local device=""
    
    # Check Intel GPU via PCI
    if [ -r "/sys/class/drm/card0/device/vendor" ]; then
        local vendor=$(read_first_line "/sys/class/drm/card0/device/vendor")
        if [ "$vendor" = "0x8086" ]; then
            device="/dev/dri/card0"
            confidence=$((confidence + 35))
            reasons="${reasons}Intel GPU detected; "
        else
            reasons="${reasons}Intel GPU not found; "
        fi
    fi
    
    # Check Intel Media SDK
    if can_load_lib "libmfx.so" || can_load_lib "libmfx.so.1"; then
        confidence=$((confidence + 50))
        reasons="${reasons}Intel Media SDK available; "
    else
        reasons="${reasons}Intel Media SDK not found; "
    fi
    
    # Check i915 driver module
    if lsmod | grep -q "i915" 2>/dev/null; then
        confidence=$((confidence + 15))
        reasons="${reasons}i915 driver loaded; "
    else
        reasons="${reasons}i915 driver not found; "
    fi
    
    # Check CPU info for Intel
    if grep -q "Intel\|GenuineIntel" /proc/cpuinfo 2>/dev/null; then
        confidence=$((confidence + 10))
        reasons="${reasons}Intel CPU detected; "
    else
        reasons="${reasons}Intel CPU not found; "
    fi
    
    local available="NO"
    if [ $confidence -ge 50 ]; then
        available="YES"
    fi
    
    echo "Available: $available"
    echo "Confidence: ${confidence}%"
    [ -n "$device" ] && echo "Device: $device"
    echo "Details: $reasons"
    echo
}

# AMD VCE Detection
detect_amd() {
    echo "Encoder: h264_amf"
    echo "Type: AMD VCE"
    
    local confidence=0
    local reasons=""
    local device=""
    
    # Check AMD GPU via PCI
    if [ -r "/sys/class/drm/card0/device/vendor" ]; then
        local vendor=$(read_first_line "/sys/class/drm/card0/device/vendor")
        if [ "$vendor" = "0x1002" ] || [ "$vendor" = "0x1022" ]; then
            device="/dev/dri/card0"
            confidence=$((confidence + 35))
            reasons="${reasons}AMD GPU detected; "
        else
            reasons="${reasons}AMD GPU NOT detected; "
        fi
    fi
    
    # Check AMD Media Framework
    if can_load_lib "libamfrt64.so" || can_load_lib "libamfrt64.so.1"; then
        confidence=$((confidence + 50))
        reasons="${reasons}AMD Media Framework available; "
    else
        reasons="${reasons}AMD Media Framework NOT found; "
    fi
    
    # Check amdgpu driver
    if lsmod | grep -q "amdgpu" 2>/dev/null; then
        confidence=$((confidence + 15))
        reasons="${reasons}amdgpu driver loaded; "
    else
        reasons="${reasons}amdgpu driver NOT found; "
    fi
    
    # Check via lspci
    if command -v lspci >/dev/null 2>&1; then
        if lspci | grep -i amd >/dev/null 2>&1; then
            confidence=$((confidence + 10))
            reasons="${reasons}AMD GPU found via lspci; "
        else
            reasons="${reasons}AMD GPU NOT found via lspci; "
        fi
    fi
    
    local available="NO"
    if [ $confidence -ge 50 ]; then
        available="YES"
    fi
    
    echo "Available: $available"
    echo "Confidence: ${confidence}%"
    [ -n "$device" ] && echo "Device: $device"
    echo "Details: $reasons"
    echo
}

# VAAPI Detection
detect_vaapi() {
    echo "Encoder: h264_vaapi"
    echo "Type: VAAPI"
    
    local confidence=0
    local reasons=""
    local device=""
    
    # Check DRI devices
    for card in /dev/dri/card*; do
        if file_exists "$card"; then
            device="$card"
            confidence=$((confidence + 25))
            reasons="${reasons}DRI device found; "
            break
        fi
    done
    
    # Check VAAPI libraries
    local lib_count=0
    if can_load_lib "libva.so" || can_load_lib "libva.so.1" || can_load_lib "libva.so.2"; then
        lib_count=$((lib_count + 1))
    fi
    if can_load_lib "libva-drm.so" || can_load_lib "libva-drm.so.1" || can_load_lib "libva-drm.so.2"; then
        lib_count=$((lib_count + 1))
    fi
    
    if [ $lib_count -ge 2 ]; then
        confidence=$((confidence + 40))
        reasons="${reasons}VAAPI libraries available; "
    elif [ $lib_count -ge 1 ]; then
        confidence=$((confidence + 20))
        reasons="${reasons}Some VAAPI libraries available; "
    fi
    
    # Check GPU drivers that support VAAPI
    if lsmod | grep -E "i915|amdgpu|nouveau" >/dev/null 2>&1; then
        confidence=$((confidence + 25))
        reasons="${reasons}VAAPI-compatible driver loaded; "
    fi
    
    # Check render device
    if file_exists "/dev/dri/renderD128"; then
        confidence=$((confidence + 10))
        reasons="${reasons}Render device available; "
    fi
    
    local available="NO"
    if [ $confidence -ge 50 ]; then
        available="YES"
    fi
    
    echo "Available: $available"
    echo "Confidence: ${confidence}%"
    [ -n "$device" ] && echo "Device: $device"
    echo "Details: $reasons"
    echo
}

# V4L2 M2M Detection
detect_v4l2() {
    echo "Encoder: h264_v4l2m2m"
    echo "Type: V4L2 M2M"
    
    local confidence=0
    local reasons=""
    local device=""
    
    # Check for V4L2 devices
    for video_dev in /dev/video*; do
        if file_exists "$video_dev"; then
            local video_num=${video_dev##*/}
            video_num=${video_num#video}
            
            # Check device name
            local name_file="/sys/class/video4linux/video${video_num}/name"
            if [ -r "$name_file" ]; then
                local name=$(read_first_line "$name_file")
                if echo "$name" | grep -iE "encode|codec|m2m" >/dev/null 2>&1; then
                    device="$video_dev"
                    confidence=$((confidence + 60))
                    reasons="${reasons}V4L2 encoder device: $name; "
                    break
                fi
            else
                # Fallback: any video device might be an encoder
                device="$video_dev"
                confidence=$((confidence + 30))
                reasons="${reasons}V4L2 device found: $video_dev; "
            fi
        fi
    done
    
    local available="NO"
    if [ $confidence -ge 50 ]; then
        available="YES"
    fi
    
    echo "Available: $available"
    echo "Confidence: ${confidence}%"
    [ -n "$device" ] && echo "Device: $device"
    echo "Details: $reasons"
    echo
}

# Main detection
echo "System Detection Results:"
echo "-------------------------"

detect_nvenc
detect_qsv
detect_amd
detect_vaapi
detect_v4l2

echo "Notes:"
echo "------"
echo "- Confidence >= 50%: Likely to work"
echo "- Confidence < 50%: May not be functional"
echo "- This script only checks system availability"
echo "- Actual encoding may require proper driver setup"
echo
echo "System Info:"
echo "------------"
echo "Kernel: $(uname -r)"
echo "Distribution: $(lsb_release -d 2>/dev/null | cut -f2 || echo "Unknown")"

if command -v lspci >/dev/null 2>&1; then
    echo "GPUs found:"
    lspci | grep -i vga | while read line; do
        echo "  $line"
    done
fi 