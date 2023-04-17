set datafile separator ","

input = 'control_2.csv'

set multiplot

# XY
set size 1.0, 0.2
set origin 0.0, 0.8
plot \
    input using 5:6 title 'xy' with lines

# XY
set size 1.0, 0.2
set origin 0.0, 0.6
plot \
    input using 1:5 title 'x' with lines, \
    input using 1:6 title 'y' with lines, \
    input using 1:7 title 'a' with lines

# WORLD VEL
set size 1.0, 0.2
set origin 0.0, 0.4
plot \
    input using 1:8 title 'w_{vx}' with lines, \
    input using 1:9 title 'w_{vy}' with lines, \
    input using 1:10 title 'w_{w}' with lines

# ROBOT VEL
set size 1.0, 0.2
set origin 0.0, 0.2
plot \
    input using 1:11 title 'r_{vx}' with lines, \
    input using 1:12 title 'r_{vy}' with lines, \
    input using 1:13 title 'r_{w}' with lines

# MOTORS
set size 1.0, 0.2
set origin 0.0, 0.0
plot \
    input using 1:14 title 'm_{1}' with lines, \
    input using 1:15 title 'm_{2}' with lines, \
    input using 1:16 title 'm_{3}' with lines

