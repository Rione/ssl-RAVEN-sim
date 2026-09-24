BUILD_DIR := build

.PHONY: all build run clean

all: build

build:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)

# QML と設定ファイルを相対パスで読むため、必ず build/ から起動する
run: build
	cd $(BUILD_DIR) && ./bin/m2-Sim

clean:
	rm -rf $(BUILD_DIR)
