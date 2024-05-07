.PHONY: help

SRC_FILES := $(shell find app/src app/lib -name '*.c' -o -name '*.h')
 
help: # show help for each of the Makefile recipes.
	@grep -E '^[a-zA-Z0-9 -]+:.*#'  Makefile | sort | while read -r l; do printf "\033[1;32m$$(echo $$l | cut -f 1 -d':')\033[00m:$$(echo $$l | cut -f 2- -d'#')\n"; done

build: # build target
	west build -b nucleo_f446re app --build-dir build

build-control: # build target winth control logs
	west build -b nucleo_f446re app --build-dir build -- -DOVERLAY_CONFIG="control_logs.conf"

rebuild: # rebuild target
	west build -b nucleo_f446re app --build-dir build --pristine

build-pc-app: # build strat pc app
	west build -b native_posix pc_app --build-dir build_pc

run-pc-app: # run strat pc app
	./build_pc/zephyr/zephyr.elf

run-pc-gui: # run start pc ui
	python tools/strat_visualizer/src/main.py

clean: # clean project
	west build -t clean --build-dir build

flash: # flash target
	west flash --build-dir build

dbg: # debug target and connect with gdb
	west debug

dbgserv: # debug target and expose gdb server
	west debugserver

format: # format all files in the project 
	clang-format -i $(SRC_FILES)
	tools/format_eof.sh $(SRC_FILES)

