#!/bin/bash
# Based on (some tweaks by me):
# https://stackoverflow.com/questions/34943632/linux-check-if-there-is-an-empty-line-at-the-end-of-a-file
# As a reminder:
# -f tests if the file is a regular file (i.e. not directory or device file)
# -s tests if the file is not 0 bytes
# -z tests if the string is null
# -c in tail outputs the last byte
# Usage:
# ./newline_test.sh $(find <root_dir> -name "*.c" -o -name "*.cpp" -o -name "*.h")
for f in "$@"
do
    if [[ -f "$f" && -s "$f" ]];
    then
        if [[ -z "$(tail -c 1 "$f")" ]];
        then
            :
        else
            echo "No newline at: $f"
        fi
    fi
done