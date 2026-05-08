set shell := ["bash", "-uc"]

OTA_AUTH := "CarInSituOTA"

default: build

build:
	pio run

lint:
	clang-format -i `find src include -name '*.cpp' -or -name '*.h' -or -name '*.ino' | xargs`

clean:
	pio run -t clean

monitor:
	pio device monitor

upload-serial:
	pio run -t upload

upload-ota target=env_var_or_default("OTA_TARGET", ""):
	#!/usr/bin/env bash
	set -euo pipefail
	if [[ -z "{{ target }}" ]]; then
		echo "No OTA_TARGET set and target not passed. Export OTA_TARGET or run: just upload-ota mycar"
		exit 1
	fi
	echo "Upload to {{ target }}"
	PLATFORMIO_UPLOAD_FLAGS="--auth={{ OTA_AUTH }}" pio run -t upload --upload-port "{{ target }}.local"

upload-ota-all:
	#!/usr/bin/env bash
	set -uo pipefail
	echo "Upload to all detected cars"
	cars=$(avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }')
	if [[ -z "${cars}" ]]; then
		echo "No cars found."
		exit 1
	fi
	echo "Cars: ${cars}"
	ok=()
	ko=()
	for car in ${cars}; do
		if PLATFORMIO_UPLOAD_FLAGS="--auth={{ OTA_AUTH }}" pio run -t upload --upload-port "${car}.local"; then
			ok+=("${car}")
		else
			ko+=("${car}")
		fi
	done
	echo "succeeded: ${ok[*]}"
	echo "failed: ${ko[*]}"
	exit "${#ko[@]}"

upload *ARGS:
	@just upload-ota {{ ARGS }}

factory-reset:
	pio run -t erase && pio run -t upload
