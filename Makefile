TARGET_URL ?= hamilton
TARGET_HOST ?= dweis@$(TARGET_URL)

.PHONY: build
build:
	cargo build --release
	cargo deb --no-build

.PHONE: install
install: build
	sudo dpkg -i PATH

.PHONY: install-dev-dependencies
install-dev-dependencies:
	cargo install cargo-deb

.PHONY: build-docker
build-docker:
	rm -rf docker_out
	mkdir docker_out
	DOCKER_BUILDKIT=1 docker build --tag hamilton-builder --file Dockerfile --output type=local,dest=docker_out .

.PHONY: deploy-with-ez-cd
deploy-with-ez-cd: build-docker
	ez-cd-cli -f docker_out/hamilton.deb -d hamilton
