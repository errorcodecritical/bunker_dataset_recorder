#!/bin/bash
set -e

# Install cuSPARSELt + PyTorch + FeatUp on first run.
# /usr/local/cuda is only available at runtime via --runtime nvidia.
FLAG="/root/.pytorch_installed"

if [ ! -f "$FLAG" ]; then
    echo "[entrypoint] Installing cuSPARSELt..."
    mkdir /tmp/cusparselt && cd /tmp/cusparselt
    CUSPARSELT_NAME="libcusparse_lt-linux-aarch64-0.7.1.0-archive"
    curl --retry 3 -OLs \
        https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/${CUSPARSELT_NAME}.tar.xz
    tar xf ${CUSPARSELT_NAME}.tar.xz
    cp -a ${CUSPARSELT_NAME}/include/* /usr/local/cuda/include/
    cp -a ${CUSPARSELT_NAME}/lib/* /usr/local/cuda/lib64/
    ldconfig
    cd / && rm -rf /tmp/cusparselt

    echo "[entrypoint] Installing PyTorch for JetPack 6.2..."
    python3 -m pip install --no-cache \
        torch==2.5.0 torchvision==0.20.0 \
        --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

    echo "[entrypoint] Installing FeatUp..."
    python3 -m pip install --no-build-isolation \
        git+https://github.com/mhamilton723/FeatUp

    touch "$FLAG"
    echo "[entrypoint] Done. PyTorch + FeatUp installed."
else
    echo "[entrypoint] PyTorch already installed, skipping."
fi

exec "$@"
