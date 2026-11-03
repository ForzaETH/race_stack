#!/bin/bash
set -eo pipefail

# Jetson only: this package doesn't exist on other images
dpkg-query -W cuda-toolkit-12-6 >/dev/null 2>&1 || exit 0

TORCH_WHEEL="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
TORCH_SHA256=6f75fd2d2ef840ede1a90dbcf40a5458214bee26cc803fa510cda2e8978d972a
CUSPARSELT="libcusparse_lt-linux-sbsa-0.6.2.3-archive"
CUSPARSELT_SHA256=512faabdf1a095796ba113a8f81921303dfa203b3ae02345fd114407e9792ad7
TORCHVISION_WHEEL="torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl"
TORCHVISION_SHA256=e7c1f76f2eb085eff8284cc71f0198d79b6bd1360646b7298f0a2b7103096dc0
TMP=$(mktemp -d)

# System libraries needed by torch and torchvision
apt-get update
apt-get install -y --no-install-recommends libopenblas-dev libjpeg-dev libpng-dev libwebp-dev
rm -rf /var/lib/apt/lists/*

# Installing OpenBLAS changes the system BLAS/LAPACK; keep what cv2 and cv_bridge already use
update-alternatives --set libblas.so.3-aarch64-linux-gnu /usr/lib/aarch64-linux-gnu/blas/libblas.so.3
update-alternatives --set liblapack.so.3-aarch64-linux-gnu /usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3

# cuSPARSELt, needed by torch; NVIDIA's install script has no branch for CUDA 12.6, so install it directly
curl -fsSL -o "$TMP/$CUSPARSELT.tar.xz" \
    "https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-sbsa/$CUSPARSELT.tar.xz"
echo "$CUSPARSELT_SHA256  $TMP/$CUSPARSELT.tar.xz" | sha256sum -c -
tar xf "$TMP/$CUSPARSELT.tar.xz" -C "$TMP"
cp -a "$TMP/$CUSPARSELT/include/." /usr/local/cuda/include/
cp -a "$TMP/$CUSPARSELT/lib/." /usr/local/cuda/lib64/
ldconfig

# NVIDIA's torch wheel; uv rejects it by default as its filename and metadata versions differ
curl -fsSL -o "$TMP/$TORCH_WHEEL" "https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/$TORCH_WHEEL"
echo "$TORCH_SHA256  $TMP/$TORCH_WHEEL" | sha256sum -c -
UV_SKIP_WHEEL_FILENAME_CHECK=1 uv pip install --system --no-cache "$TMP/$TORCH_WHEEL"

# Ultralytics' torchvision built for this torch; NVIDIA publishes none and PyPI's has no Orin GPU code
curl -fsSL -o "$TMP/$TORCHVISION_WHEEL" \
    "https://github.com/ultralytics/assets/releases/download/v0.0.0/$TORCHVISION_WHEEL"
echo "$TORCHVISION_SHA256  $TMP/$TORCHVISION_WHEEL" | sha256sum -c -
uv pip install --system --no-cache --no-deps "$TMP/$TORCHVISION_WHEEL"

rm -rf "$TMP"
