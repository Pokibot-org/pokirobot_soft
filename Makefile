.PHONY: help

SRC_FILES := $(shell find app/src app/lib -name '*.c' -o -name '*.h')
 
help: # show help for each of the Makefile recipes.
	@grep -E '^[a-zA-Z0-9 -]+:.*#'  Makefile | sort | while read -r l; do printf "\033[1;32m$$(echo $$l | cut -f 1 -d':')\033[00m:$$(echo $$l | cut -f 2- -d'#')\n"; done

build: # build target
	west build -b nucleo_f446re app

rebuild: # rebuild target
	west build -b nucleo_f446re app --pristine

build-pc-app: # build strat pc app
	west build -b native_posix pc_app --build-dir build-pc-app

run-pc-app: # run strat pc app
	./build-pc-app/zephyr/zephyr.elf

run-pc-gui: # run start pc ui
	python tools/strat_visualizer/src/main.py

clean: # clean project
	west build -t clean

flash: # flash target
	west flash

dbg: # debug target and connect with gdb
	west debug

dbgserv: # debug target and expose gdb server
	west debugserver

format: # format all files in the project 
	clang-format -i $(SRC_FILES)
	tools/format_eof.sh $(SRC_FILES)

