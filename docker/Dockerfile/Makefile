SHELL := /bin/bash

SPEED ?= 8
USE_PROXY ?= true
PROXY_PORT ?= 7897
TENSORRT_TARBALL ?= TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz
TENSORRT_PATH ?= ../$(TENSORRT_TARBALL)
BUILD_NETWORK ?= host
JETSON_PLATFORM ?= linux/arm64
JETSON_IMAGE ?= fastdronexi35:orin
RK3588_PLATFORM ?=
RK3588_IMAGE ?= fastdronexi35:rk3588
RK3588_DOCKER_BUILDKIT ?= 0
PC_IMAGE ?= fastdronexi35:pc
PC_SIM_BASE_IMAGE ?= $(PC_IMAGE)
PC_SIM_IMAGE ?= fastdronexi35:pc-sim
DOCKER_BUILDKIT ?= 1
BUILDKIT_PROGRESS ?= auto
DEFAULT_PROXY_HOST ?= $(shell if grep -qi microsoft /proc/version 2>/dev/null && getent hosts host.docker.internal >/dev/null 2>&1; then printf '%s' host.docker.internal; else printf '%s' 127.0.0.1; fi)
DEFAULT_CONTAINER_HTTP_PROXY ?= http://$(DEFAULT_PROXY_HOST):$(PROXY_PORT)
DEFAULT_CONTAINER_HTTPS_PROXY ?= http://$(DEFAULT_PROXY_HOST):$(PROXY_PORT)
CONTAINER_HTTP_PROXY ?= $(DEFAULT_CONTAINER_HTTP_PROXY)
CONTAINER_HTTPS_PROXY ?= $(DEFAULT_CONTAINER_HTTPS_PROXY)

ifeq ($(filter $(USE_PROXY),true false),)
$(error USE_PROXY must be true or false)
endif

PROXY_BUILD_ARGS := --build-arg USE_PROXY=$(USE_PROXY) \
	--build-arg BUILD_HTTP_PROXY=$(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTP_PROXY),) \
	--build-arg BUILD_HTTPS_PROXY=$(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTPS_PROXY),)
COMMON_BUILD_ARGS := --network=$(BUILD_NETWORK) --build-arg USE_PROC=$(SPEED) $(PROXY_BUILD_ARGS)
JETSON_PLATFORM_ARG := $(if $(JETSON_PLATFORM),--platform=$(JETSON_PLATFORM),)
RK3588_PLATFORM_ARG := $(if $(RK3588_PLATFORM),--platform=$(RK3588_PLATFORM),)
DOCKER_BUILD_ENV := DOCKER_BUILDKIT=$(DOCKER_BUILDKIT) BUILDKIT_PROGRESS=$(BUILDKIT_PROGRESS)
BUILDKIT_PROGRESS_ARG := $(if $(filter 1,$(DOCKER_BUILDKIT)),--progress=$(BUILDKIT_PROGRESS),)

.PHONY: check_buildkit check_tensorrt check_pc_image jetson jetson_rebuild rk3588 rk3588_rebuild pc pc_sim rm_jetson

check_buildkit:
	@if [ "$(DOCKER_BUILDKIT)" = "1" ] && ! docker buildx version >/dev/null 2>&1; then \
		echo "BuildKit is enabled, but docker buildx is missing or broken."; \
		echo "Install the docker buildx plugin, or run with DOCKER_BUILDKIT=0 to use the legacy builder."; \
		exit 1; \
	fi

check_tensorrt:
	@if [ ! -f "$(TENSORRT_PATH)" ]; then \
		echo "Missing $(TENSORRT_PATH)"; \
		echo "Please download $(TENSORRT_TARBALL) manually and put it in the parent directory."; \
		exit 1; \
	fi

check_pc_image:
	@if ! docker image inspect "$(PC_SIM_BASE_IMAGE)" >/dev/null 2>&1; then \
		echo "Missing base image: $(PC_SIM_BASE_IMAGE)"; \
		echo "Build it first with: make pc"; \
		exit 1; \
	fi

jetson: check_buildkit
	@$(DOCKER_BUILD_ENV) docker build $(BUILDKIT_PROGRESS_ARG) $(JETSON_PLATFORM_ARG) $(COMMON_BUILD_ARGS) -t $(JETSON_IMAGE) -f ./Dockerfile.jetson ..

jetson_rebuild: check_buildkit
	@$(DOCKER_BUILD_ENV) docker build $(BUILDKIT_PROGRESS_ARG) --no-cache $(JETSON_PLATFORM_ARG) $(COMMON_BUILD_ARGS) -t $(JETSON_IMAGE) -f ./Dockerfile.jetson ..

rk3588:
	@DOCKER_BUILDKIT=$(RK3588_DOCKER_BUILDKIT) docker build $(RK3588_PLATFORM_ARG) $(COMMON_BUILD_ARGS) -t $(RK3588_IMAGE) -f ./Dockerfile.rk3588 ..

rk3588_rebuild:
	@DOCKER_BUILDKIT=$(RK3588_DOCKER_BUILDKIT) docker build --no-cache $(RK3588_PLATFORM_ARG) $(COMMON_BUILD_ARGS) -t $(RK3588_IMAGE) -f ./Dockerfile.rk3588 ..

pc: check_buildkit check_tensorrt
	@echo "Use proxy: $(USE_PROXY)"; \
	echo "HTTP proxy: $(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTP_PROXY),)"; \
	echo "HTTPS proxy: $(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTPS_PROXY),)"; \
	$(DOCKER_BUILD_ENV) docker build $(BUILDKIT_PROGRESS_ARG) $(COMMON_BUILD_ARGS) -t $(PC_IMAGE) -f ./Dockerfile.pc \
		..

pc_sim: check_buildkit check_pc_image
	@echo "Base image: $(PC_SIM_BASE_IMAGE)"; \
	echo "Simulation image: $(PC_SIM_IMAGE)"; \
	echo "Use proxy: $(USE_PROXY)"; \
	$(DOCKER_BUILD_ENV) docker build $(BUILDKIT_PROGRESS_ARG) \
		--network=$(BUILD_NETWORK) \
		$(PROXY_BUILD_ARGS) \
		--build-arg PC_BASE_IMAGE=$(PC_SIM_BASE_IMAGE) \
		-t $(PC_SIM_IMAGE) \
		-f ./Dockerfile.pc_sim \
		..

rm_jetson:
	@docker rmi $(JETSON_IMAGE)
