# local_sensing_node

This package uses CUDA to render depth images from the simulator point cloud.

## CUDA Architecture

The CUDA kernels must be compiled for the GPU architecture of the machine that runs
`pcl_render_node`. If the package is compiled for the wrong architecture, the node
may fail with an error like:

```text
CUDA depth render failed: CudaException: depth_initial kernel failed
cudaError code: no kernel image is available for execution on the device
```

When changing platforms, rebuild this package with the correct
`LOCAL_SENSING_CUDA_ARCH` value:

```bash
# Orin NX
catkin_make --pkg local_sensing_node -DLOCAL_SENSING_CUDA_ARCH=87

# RTX 20 / T4
catkin_make --pkg local_sensing_node -DLOCAL_SENSING_CUDA_ARCH=75

# RTX 30
catkin_make --pkg local_sensing_node -DLOCAL_SENSING_CUDA_ARCH=86

# RTX 40
catkin_make --pkg local_sensing_node -DLOCAL_SENSING_CUDA_ARCH=89
```

To build one binary that supports multiple platforms, pass multiple architectures:

```bash
catkin_make --pkg local_sensing_node -DLOCAL_SENSING_CUDA_ARCH="75;86;87"
```

This increases build time and binary size, but avoids recompiling when moving
between those GPU platforms.
