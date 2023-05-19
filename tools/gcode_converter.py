#!/usr/bin/env python3

import sys
import re

G_REGEX = re.compile(r"(G\d+) X(\d+) Y(\d+);")

def convert_gcode(lines):
    _write = False
    scaling = 3
    i = 0
    print("int wps_len;")
    print("pen_up();")
    print("pen_down();")
    print("pen_up();")
    print()
    print("// drawing start")
    print(f"pos2_t wps{i}[] = {{")
    for l in lines:
        m = re.match(G_REGEX, l)
        if not m:
            continue
        if m.group(1) == "G00":
            if _write:
                print("};")
                print(f"wps_len = ARRAY_SIZE(wps{i});")
                print(f"control_set_waypoints(&shared_ctrl, wps{i}, wps_len);")
                print("control_task_wait_target_default(100000.0f, 10.0f);")
                print()
                i += 1
                print("pen_up();")
                print(f"pos2_t wps{i}[] = {{")
                # TODO pen up and end group
                pass
            _write = False
            print(f"(pos2_t){{.x = {int(m.group(2)) * scaling}, .y = {int(m.group(3)) * scaling}, .a = 0.0f}},")
        elif m.group(1) == "G01":
            if not _write:
                print("};")
                print(f"wps_len = ARRAY_SIZE(wps{i});")
                print(f"control_set_waypoints(&shared_ctrl, wps{i}, wps_len);")
                print("control_task_wait_target_default(100000.0f, 10.0f);")
                print()
                i += 1
                print("pen_down();")
                print(f"pos2_t wps{i}[] = {{")
                pass
            _write = True
            print(f"(pos2_t){{.x = {int(m.group(2)) * scaling}, .y = {int(m.group(3)) * scaling}, .a = 0.0f}},")
        else:
            continue
    print("};")
    print(f"wps_len = ARRAY_SIZE(wps{i});")
    print("pen_up();")
    print(f"control_set_waypoints(&shared_ctrl, wps{i}, wps_len);")
    print("control_task_wait_target_default(100000.0f, 10.0f);")
    print()
    i += 1
    print("// drawing end")
    print()
    print(f"pos2_t wps{i}[] = {{")
    print(f"(pos2_t){{.x = 0.0f, .y = 0.0f, .a = -2.0f}},")
    print("};")
    print(f"control_set_waypoints(&shared_ctrl, wps{i}, wps_len);")
    print("control_task_wait_target_default(15.0f, 10.0f);")

if __name__ == "__main__":
    with open(sys.argv[1], "r") as f:
        drawing = convert_gcode(f.readlines())
    drawing
