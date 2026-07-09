#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
OPENCV_VERSION="${OPENCV_VERSION:-4.5.4}"
VISION_OPENCV_VERSION="${VISION_OPENCV_VERSION:-1.16.2}"
CUDA_ARCH_BIN="${CUDA_ARCH_BIN:-8.7}" # Orin: 8.7, Xavier: 7.2
USE_PROC="${USE_PROC:-$(nproc)}"
ENABLE_NEON="${ENABLE_NEON:-ON}"
SKIP_OPENCV_BUILD="${SKIP_OPENCV_BUILD:-0}"

export DEBIAN_FRONTEND=noninteractive

echo "[1/5] Reinstalling OpenCV ${OPENCV_VERSION}"
echo "      ROS_DISTRO=${ROS_DISTRO_NAME}"
echo "      CUDA_ARCH_BIN=${CUDA_ARCH_BIN}"
echo "      USE_PROC=${USE_PROC}"
echo "      SKIP_OPENCV_BUILD=${SKIP_OPENCV_BUILD}"

rm -rf "/root/cv_bridge${OPENCV_VERSION}"
rm -rf "/root/cv_bridge${OPENCV_VERSION}_ws"

if [[ "${SKIP_OPENCV_BUILD}" != "1" ]]; then
rm -rf "/usr/local/opencv-${OPENCV_VERSION}"
rm -rf /tmp/opencv-build

mkdir -p /tmp/opencv-build/boostdesc_extfiles
cd /tmp/opencv-build

echo "[2/5] Downloading OpenCV third-party files"
for file in boostdesc_bgm.i boostdesc_bgm_bi.i boostdesc_bgm_hd.i \
            boostdesc_binboost_064.i boostdesc_binboost_128.i boostdesc_binboost_256.i \
            boostdesc_lbgm.i; do
  wget -q --tries=5 --waitretry=5 --timeout=30 --read-timeout=30 --retry-connrefused \
    "https://raw.githubusercontent.com/opencv/opencv_3rdparty/contrib_xfeatures2d_boostdesc_20161012/${file}" \
    -O "boostdesc_extfiles/${file}"
done

for file in vgg_generated_48.i vgg_generated_64.i vgg_generated_80.i vgg_generated_120.i; do
  wget -q --tries=5 --waitretry=5 --timeout=30 --read-timeout=30 --retry-connrefused \
    "https://raw.githubusercontent.com/opencv/opencv_3rdparty/contrib_xfeatures2d_vgg_20160317/${file}" \
    -O "boostdesc_extfiles/${file}"
done

echo "[3/5] Downloading OpenCV sources"
for attempt in 1 2 3 4 5; do
  git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv_contrib.git opencv_contrib && break
  rm -rf opencv_contrib
  sleep 5
done
test -d opencv_contrib
cp boostdesc_extfiles/*.i opencv_contrib/modules/xfeatures2d/src/

wget -q --tries=5 --waitretry=5 --timeout=30 --read-timeout=30 --retry-connrefused \
  "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip" -O opencv.zip
unzip -q opencv.zip
rm opencv.zip

echo "[4/5] Building OpenCV ${OPENCV_VERSION}"
cd "opencv-${OPENCV_VERSION}"
mkdir -p build
cd build

cmake .. \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX="/usr/local/opencv-${OPENCV_VERSION}" \
  -D WITH_CUDA=ON \
  -D WITH_CUDNN=ON \
  -D WITH_CUBLAS=ON \
  -D CUDNN_VERSION='8.6' \
  -D CUDNN_INCLUDE_DIR=/usr/include/ \
  -D CUDA_ARCH_BIN="${CUDA_ARCH_BIN}" \
  -D CUDA_ARCH_PTX="" \
  -D CUDA_FAST_MATH=ON \
  -D WITH_TBB=ON \
  -D BUILD_opencv_python2=OFF \
  -D BUILD_opencv_python3=ON \
  -D OPENCV_DNN_CUDA=ON \
  -D OPENCV_ENABLE_NONFREE=ON \
  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_java=OFF \
  -D BUILD_opencv_python=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_opencv_apps=OFF \
  -D ENABLE_NEON="${ENABLE_NEON}" \
  -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
  -D WITH_EIGEN=ON \
  -D WITH_IPP=OFF \
  -D WITH_OPENCL=OFF \
  -D PYTHON3_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
  -D BUILD_LIST="calib3d,features2d,highgui,dnn,imgproc,imgcodecs,photo,cudev,cudaoptflow,cudaimgproc,cudalegacy,cudaarithm,cudacodec,cudastereo,cudafeatures2d,xfeatures2d,tracking,stereo,aruco,videoio,ccalib"

make -j"${USE_PROC}"
make install
ldconfig
else
  if [[ ! -f "/usr/local/opencv-${OPENCV_VERSION}/lib/cmake/opencv4/OpenCVConfig.cmake" ]]; then
    echo "SKIP_OPENCV_BUILD=1 was set, but /usr/local/opencv-${OPENCV_VERSION} is not installed." >&2
    exit 1
  fi
  echo "Skipping OpenCV build. Found /usr/local/opencv-${OPENCV_VERSION}/lib/cmake/opencv4/OpenCVConfig.cmake"
fi

echo "[5/5] Rebuilding cv_bridge_454 against OpenCV ${OPENCV_VERSION}"
cd /root
wget -q --tries=5 --waitretry=5 --timeout=30 --read-timeout=30 --retry-connrefused \
  "https://github.com/ros-perception/vision_opencv/archive/refs/tags/${VISION_OPENCV_VERSION}.zip" \
  -O /tmp/vision_opencv.zip

unzip -q /tmp/vision_opencv.zip -d /root
mv "/root/vision_opencv-${VISION_OPENCV_VERSION}/cv_bridge" "/root/cv_bridge${OPENCV_VERSION}"
rm -rf "/root/vision_opencv-${VISION_OPENCV_VERSION}" /tmp/vision_opencv.zip

sed -i 's|^project(cv_bridge)$|#project(cv_bridge)\nproject(cv_bridge_454)|' \
  "/root/cv_bridge${OPENCV_VERSION}/CMakeLists.txt"

sed -i '/^set(_opencv_version 4)/i set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)#use self-opencv' \
  "/root/cv_bridge${OPENCV_VERSION}/CMakeLists.txt"

sed -i '/^  set(_opencv_version 3)$/a else()\n  message(WARNING "CV_Bridge use OpenCV_VERSION: ${OpenCV_VERSION}")\n  message(WARNING "OpenCV Path is: ${OpenCV_INSTALL_PATH}")' \
  "/root/cv_bridge${OPENCV_VERSION}/CMakeLists.txt"

sed -i 's|<name>cv_bridge</name>|<name>cv_bridge_454</name>|' \
  "/root/cv_bridge${OPENCV_VERSION}/package.xml"

mkdir -p "/root/cv_bridge${OPENCV_VERSION}_ws/src"
mv "/root/cv_bridge${OPENCV_VERSION}" "/root/cv_bridge${OPENCV_VERSION}_ws/src"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
cd "/root/cv_bridge${OPENCV_VERSION}_ws"
catkin_make

grep -qxF "source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash" /root/.bashrc || \
  echo "source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash" >> /root/.bashrc

echo
echo "Done."
echo "OpenCV install:"
test -f "/usr/local/opencv-${OPENCV_VERSION}/lib/cmake/opencv4/OpenCVConfig.cmake" && \
  echo "  /usr/local/opencv-${OPENCV_VERSION}/lib/cmake/opencv4/OpenCVConfig.cmake"
find "/usr/local/opencv-${OPENCV_VERSION}/lib" -maxdepth 1 -name 'libopencv_core.so*' | sort | tail -1
echo
echo "Next shell:"
echo "  source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash"
