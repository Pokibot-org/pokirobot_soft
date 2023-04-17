
.PHONY: help build clean format

SRC_FILES := $(shell find app/src app/lib -name '*.c' -o -name '*.h')
 
help:
	@echo "you are on your own you fool"
	@echo "make <help / build / rebuild / clean / format>"

build:
	west build -b nucleo_f446re app

rebuild:
	west build -b nucleo_f446re app --pristine

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

