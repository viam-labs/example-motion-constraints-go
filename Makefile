GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/example-motion-constraints-go
VERSION := $(shell cat VERSION 2>/dev/null)
PLATFORM ?= linux/amd64

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go
	$(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

.PHONY: lint
lint:
	gofmt -s -w .

.PHONY: update
update:
	go get go.viam.com/rdk@latest
	go mod tidy

.PHONY: test
test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY) VERSION assets/*.ply
	tar czf $@ meta.json $(MODULE_BINARY) assets/

# Regenerate the per-arm text-PLY labels using matplotlib + shapely +
# trimesh. Edit scripts/generate_text_assets.py's LABELS list to add
# or change strings, then run `make assets`.
#
# Uses the sibling example-visualizations-python venv since it already
# has the right packages installed. Swap to your own venv with the
# same deps (matplotlib shapely trimesh mapbox_earcut) as needed.
.PHONY: assets
assets:
	~/viam/example-visualizations-python/.venv/bin/python scripts/generate_text_assets.py

.PHONY: module
module: test module.tar.gz

.PHONY: all
all: test module.tar.gz

.PHONY: upload
upload: module.tar.gz
	viam module upload --version=$(VERSION) --platform=$(PLATFORM) module.tar.gz

# Build + upload for all supported platforms in one shot.
# Only run this once the module has been validated locally on at least one platform.
.PHONY: upload-all
upload-all:
	$(MAKE) upload PLATFORM=linux/amd64
	$(MAKE) upload PLATFORM=linux/arm64

.PHONY: setup
setup:
	go mod tidy
