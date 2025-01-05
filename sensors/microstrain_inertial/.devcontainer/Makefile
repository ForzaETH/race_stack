.PHONY: all image build build-shell deploy-tag deploy-latest _deploy clean

# These variables should be overridden in CI builds to configure this build
version ?= latest
docker ?= docker
arch ?= amd64
ros_version ?= galactic

# Just set some directory values to support out of tree builds
makefile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
docker_dir := $(dir $(makefile_path))
project_dir := $(abspath $(docker_dir)/..)
project_name := $(shell echo $(notdir $(project_dir)) | tr A-Z a-z)
artifacts_dir := $(docker_dir)/.artifacts/$(arch)-$(ros_version)
build_dir := $(artifacts_dir)/build
install_dir := $(artifacts_dir)/install
docker_catkin_root := /home/microstrain/catkin_ws
docker_catkin_src_dir := $(docker_catkin_root)/src
docker_catkin_build_dir := $(docker_catkin_root)/build
docker_catkin_install_dir := $(docker_catkin_root)/install

# All of these directories have to exist before we can run most tasks
dir_deps := $(artifacts_dir) $(build_dir) $(install_dir)

# Grab the user and group ID so that the files we create can be deleted and managed from the base system
user_id := $(shell id -u)
group_id := $(shell id -g)

# Set up some variables for the docker build
dev_dockerfile := $(docker_dir)/Dockerfile.dev
build_args := --build-arg http_proxy --build-arg https_proxy --build-arg no_proxy --build-arg ARCH=$(arch) --build-arg ROS_VERSION=$(ros_version) --build-arg USER_ID=$(user_id) --build-arg GROUP_ID=$(group_id)
run_args := -e http_proxy -e https_proxy -e no_proxy
run_mounts := -v "$(project_dir):$(docker_catkin_src_dir)" -v "$(build_dir):$(docker_catkin_build_dir)" -v "$(install_dir):$(docker_catkin_install_dir)"
dev_image_name := $(arch)/$(project_name)-dev:$(ros_version)
dev_image_artifact := $(artifacts_dir)/.image

# Runtime image stuff
docker_org := microstrain
driver_name := microstrain_inertial_driver
driver_version := $(shell cat $(project_dir)/$(driver_name)/package.xml | grep -E '<version>.*</version>' | cut -d'>' -f2 | cut -d'<' -f1)
driver_dockerfile := $(docker_dir)/Dockerfile.driver
driver_image_artifact := $(artifacts_dir)/.image-$(version)-driver
driver_image_name_no_version := $(docker_org)/ros-$(driver_name)
driver_image_name := $(driver_image_name_no_version):$(version)-$(ros_version)-$(arch)

# All supported ROS versions, and architectures
latest_tag := ros2
latest_ros_version := rolling
all_ros_version := foxy galactic rolling
all_arch := amd64 arm64v8

all: image

# Meant to be called on "tags" of this repo to create versioned images
deploy-tag:
	$(MAKE) --no-print-directory docker=$(docker) version=$(driver_version) _deploy
	@latest_manifest_flags=""; \
	for arch in $(all_arch); do \
		# Keep track of all the image names so we can make a manifest later \
		image_name="$$($(MAKE) --no-print-directory version=$(driver_version) docker=$(docker) arch=$${arch} ros_version=$(latest_ros_version) driver-image-name)"; \
		latest_manifest_flags="$${latest_manifest_flags} --amend $${image_name}"; \
	done || exit 1; \
	\
	# Tag this version number to the latest ros version \
	versioned_image="$(driver_image_name_no_version):$(driver_version)"; \
	$(docker) manifest rm "$${versioned_image}" || true; \
	$(docker) manifest create "$${versioned_image}" $${latest_manifest_flags}; \
	$(docker) manifest push "$${versioned_image}"; \


# Meant to be called on the "ros" branch every time it is pushed to to create "latest" images
deploy-latest:
	$(MAKE) --no-print-directory docker=$(docker) version=latest _deploy
	@latest_manifest_flags=""; \
	for arch in $(all_arch); do \
		# Keep track of all the image names so we can make a manifest later \
		image_name="$$($(MAKE) --no-print-directory version=latest docker=$(docker) arch=$${arch} ros_version=$(latest_ros_version) driver-image-name)"; \
		latest_manifest_flags="$${latest_manifest_flags} --amend $${image_name}"; \
	done || exit 1; \
	\
	# Tag the latest ROS version to the latest tag \
	latest_tagged_image="$(driver_image_name_no_version):$(latest_tag)"; \
	$(docker) manifest rm "$${latest_tagged_image}" || true; \
	$(docker) manifest create "$${latest_tagged_image}" $${latest_manifest_flags}; \
	$(docker) manifest push "$${latest_tagged_image}"; \
	\
	# Use the latest ROS2 image as our latest tag \
	latest_image="$(driver_image_name_no_version):latest"; \
	$(docker) manifest rm $${latest_image} || true; \
	$(docker) manifest create $${latest_image} $${latest_manifest_flags}; \
	$(docker) manifest push $${latest_image};

# Abstract deploy task meant to be called by the above tasks, not manually
_deploy:
	@for ros_version in $(all_ros_version); do \
		driver_manifest_flags=""; \
		for arch in $(all_arch); do \
			# Keep track of all the image names so we can make a manifest later \
			image_name="$$($(MAKE) --no-print-directory version=$(version) docker=$(docker) arch=$${arch} ros_version=$${ros_version} driver-image-name)"; \
			\
			# Build the image, and add it's name to a list so we can make the manifest \
			$(MAKE) --no-print-directory version=$(version) docker=$(docker) arch=$${arch} ros_version=$${ros_version} image || exit 1; \
			driver_manifest_flags="$${driver_manifest_flags} --amend $${image_name}"; \
			\
			# Push the image to dockerhub. This requires you to be logged in \
			$(docker) push $${image_name} || exit 1; \
		done || exit 1; \
		\
		# Create the manifest and push it \
		driver_manifest_name="$(driver_image_name_no_version):$(version)-$${ros_version}"; \
		$(docker) manifest rm "$${driver_manifest_name}" || true; \
		$(docker) manifest create "$${driver_manifest_name}" $${driver_manifest_flags} || exit 1; \
		$(docker) manifest push "$${driver_manifest_name}" || exit 1; \
	done || exit 1;

image: $(driver_image_artifact)

driver-image-name:
	@echo $(driver_image_name)

$(driver_image_artifact): $(driver_dockerfile) $(dev_image_artifact) | $(dir_deps)
	@$(docker) build \
		--build-arg DEV_IMAGE="$(dev_image_name)" \
		-t $(driver_image_name) \
		-f $(driver_dockerfile) \
		$(build_args) \
		$(project_dir)
	@echo $(driver_image_name) > $@

$(dev_image_artifact): $(dev_dockerfile) | $(dir_deps)
	@$(docker) build \
		-t $(dev_image_name) \
		-f $(dev_dockerfile) \
		$(build_args) \
		$(project_dir)
	@echo $(dev_image_name) > $@

build-shell: $(dev_image_artifact)
	@$(docker) run \
		-it \
		--rm \
		-v /dev:/dev \
		--user "microstrain" \
		-w $(docker_catkin_root) \
		--net="host" \
		--privileged \
		--entrypoint="/bin/bash" \
		$(run_args) \
		$(run_mounts) \
		$$(cat $<) -c " \
			sudo apt-get update; \
			rosdep install --from-paths $(docker_catkin_src_dir) --ignore-src -r -y; \
			/bin/bash; \
		"

$(artifacts_dir):
	@mkdir -p $@

$(build_dir):
	@mkdir -p $@

$(install_dir):
	@mkdir -p $@

clean:
	@rm -rf "$(build_dir)" "$(install_dir)"
	@rm -f "$(dev_image_artifact)"
	@docker ps -a | grep "$(project_name)" | grep "$(arch)" | grep "$(ros_version)" | tr -s " " | cut -d' ' -f1 | xargs docker rm -f || echo "No containers to remove"
	@docker images | grep "$(project_name)" | grep "$(arch)" | grep "$(ros_version)" | tr -s " " | cut -d' ' -f1 | xargs docker rmi -f || echo "No images to remove"
