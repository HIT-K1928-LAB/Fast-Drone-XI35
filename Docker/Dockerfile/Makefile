SHELL := /bin/bash

SPEED ?= 32
USE_PROXY ?= true
PROXY_PORT ?= 7897
TENSORRT_TARBALL ?= TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz
TENSORRT_PATH ?= ../$(TENSORRT_TARBALL)
BUILD_NETWORK ?= host
JETSON_PLATFORM ?= linux/arm64
JETSON_BASE_IMAGE ?= fastdronexi35:orin_base_35.3.1
JETSON_IMAGE ?= fastdronexi35:orin
PC_IMAGE ?= fastdronexi35:pc
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

.PHONY: check_tensorrt check_jetson_base jetson_base jetson jetson_rebuild pc rm_jetson

check_tensorrt:
	@if [ ! -f "$(TENSORRT_PATH)" ]; then \
		echo "Missing $(TENSORRT_PATH)"; \
		echo "Please download $(TENSORRT_TARBALL) manually and put it in the parent directory."; \
		exit 1; \
	fi

check_jetson_base:
	@if ! docker image inspect "$(JETSON_BASE_IMAGE)" >/dev/null 2>&1; then \
		echo "Missing base image $(JETSON_BASE_IMAGE)"; \
		echo "Please run: make jetson_base"; \
		exit 1; \
	fi

jetson_base:
	@docker build $(JETSON_PLATFORM_ARG) $(COMMON_BUILD_ARGS) -t $(JETSON_BASE_IMAGE) -f ./Dockerfile.jetson_base ..

jetson: check_jetson_base
	@docker build $(JETSON_PLATFORM_ARG) $(COMMON_BUILD_ARGS) --build-arg JETSON_BASE_IMAGE=$(JETSON_BASE_IMAGE) -t $(JETSON_IMAGE) -f ./Dockerfile.jetson ..

jetson_rebuild: check_jetson_base
	@docker build --no-cache $(JETSON_PLATFORM_ARG) $(COMMON_BUILD_ARGS) --build-arg JETSON_BASE_IMAGE=$(JETSON_BASE_IMAGE) -t $(JETSON_IMAGE) -f ./Dockerfile.jetson ..

pc: check_tensorrt
	@echo "Use proxy: $(USE_PROXY)"; \
	echo "HTTP proxy: $(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTP_PROXY),)"; \
	echo "HTTPS proxy: $(if $(filter true,$(USE_PROXY)),$(CONTAINER_HTTPS_PROXY),)"; \
	docker build $(COMMON_BUILD_ARGS) -t $(PC_IMAGE) -f ./Dockerfile.pc \
		..

rm_jetson:
	@docker rmi $(JETSON_IMAGE)
