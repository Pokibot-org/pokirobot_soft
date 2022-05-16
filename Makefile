
.PHONY: help build clean format

SRC_FILES := $(shell find src lib -name '*.c' -o -name '*.h')
 
help:
	@echo "you are on your own you fool"
	@echo "make <help / build / clean / format>"

build:
	west build -b nucleo_f446re

rebuild:
	west build -b nucleo_f446re --pristine

clean:
	west build -t clean

flash:
	west flash

dbg:
	west debug

dbgserv:
	west debugserver

format:
	clang-format -i $(SRC_FILES)
	tools/format_eof.sh $(SRC_FILES)

