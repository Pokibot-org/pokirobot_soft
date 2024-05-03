#!/usr/bin/env python
""" pygame.examples.moveit

This is the full and final example from the Pygame Tutorial,
"How Do I Make It Move". It creates 10 objects and animates
them on the screen.

It also has a separate player character that can be controlled with arrow keys.

Note it's a bit scant on error checking, but it's easy to read. :]
Fortunately, this is python, and we needn't wrestle with a pile of
error codes.
"""
import os
import pygame as pg
import threading
import socketserver
import json
import math

main_dir = os.path.split(os.path.abspath(__file__))[0]


class Board:
    def __init__(self) -> None:
        self.real_size = [2000, 3000]
        self.dim_x = None
        self.dim_y = None


class Robot:
    def __init__(self) -> None:
        self.radius = 190
        self.pos = [0, 0]
        self.dir = 0


robot = Robot()


# quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, "../img", name)
    return pg.image.load(path).convert()


def draw_robot(screen, board, robot):
    ratio = board.dim_x[1] / board.real_size[1]
    on_board_radius = robot.radius * ratio
    on_board_pos = (robot.pos[0] * ratio, board.dim_y[1] - robot.pos[1] * ratio)

    pg.draw.circle(screen, (30, 30, 100), on_board_pos, on_board_radius)
    pg.draw.line(
        screen,
        (130, 30, 30),
        on_board_pos,
        (
            on_board_pos[0] + on_board_radius * math.cos(robot.dir),
            on_board_pos[1] + on_board_radius * math.sin(robot.dir + math.pi),
        ),
        width=4,
    )


# here's the full code
def main():
    clock = pg.time.Clock()
    screen = pg.display.set_mode((400, 300), pg.RESIZABLE)

    raw_background = load_image("vinyle.png")
    raw_background_ratio = raw_background.get_width() / raw_background.get_height()
    # pg.display.set_caption("test")

    board = Board()
    # This is a simple event handler that enables player input.
    while True:
        info = pg.display.Info()
        screen.fill(pg.Color(0, 0, 0))
        background = pg.transform.scale(
            raw_background, (info.current_h * raw_background_ratio, info.current_h)
        )
        board.dim_x = [0, background.get_width()]
        board.dim_y = [0, background.get_height()]
        screen.blit(background, (board.dim_x[0], board.dim_y[0]))

        draw_robot(screen, board, robot)
        # Get all keys currently pressed, and move when an arrow key is held.
        # keys = pg.key.get_pressed()
        # if keys[pg.ESC]:
        #     return

        for e in pg.event.get():
            # quit upon screen exit
            if e.type == pg.QUIT:
                return

        clock.tick(60)
        pg.display.update()
        pg.time.delay(100)


class Session(socketserver.BaseRequestHandler):
    def __init__(self, *args, **keys):
        super().__init__(*args, **keys)

    def handle(self):
        """Handle proto"""
        try:
            json_data = json.loads(self.request[0].decode())
            if "pos" in json_data:
                robot.pos[0] = json_data["pos"]["x"] + 1500
                robot.pos[1] = json_data["pos"]["y"]
                robot.dir = json_data["pos"]["a"]
                print(robot.pos)
        except Exception as err:
            print("Error during decoding", err)
            print(self.request[0])


if __name__ == "__main__":
    socket_name = "/tmp/strat_visu_server.sock"
    try:
        os.remove(socket_name)
    except:
        pass

    serv = socketserver.UnixDatagramServer(socket_name, Session, bind_and_activate=True)
    serv_thread = threading.Thread(target=serv.serve_forever)
    serv_thread.start()
    pg.init()
    main()
    pg.quit()
