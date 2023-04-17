#"!/bin/bash

# from https://github.com/faasm/faasm/issues/318
# Based on (some tweaks by me):
# https://stackoverflow.com/questions/34943632/linux-check-if-there-is-an-empty-line-at-the-end-of-a-file
# As a reminder:
# -f tests if the file is a regular file (i.e. not directory or device file)
# -s tests if the file is not 0 bytes
# -z tests if the string is null
# -c in tail outputs the last byte
# Usage:
# ./newline_test.sh $(find <root_dir> -name '*.c' -o -name '*.cpp' -o -name '*.h')

for FILE in "$@"
do
    if [[ -f "${FILE}" && -s "${FILE}" ]];
    then
        if [[ -z "$(tail -c 1 ${FILE})" ]];
        then
            :
        else
            echo "inserting EOF: ${FILE}"
            echo >> "${FILE}"
        fi
    fi
done

