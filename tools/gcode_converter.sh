#"!/bin/bash

echo 'pos2_t draw_wps[] = {'
cat $1 | \
    grep G01 | \
    sed -E \
        -e 's#G01#    (pos2_t){#' \
        -e 's#X([0-9]+)#.x = \1.0f, #' \
        -e 's#Y([0-9]+)#.y = \1.0f, #' \
        -e 's#;#.a = 0.0f},#'
echo '};'

LEN=`cat $1 | grep -c G01`
echo "int draw_wps_len = $LEN;"
