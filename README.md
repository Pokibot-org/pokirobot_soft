# pokirobot_soft
Soft du robot pokibot utilisé pour la coupe de france de robotique.

```sh
# create a new west workspace and pull the firmware
west init -m https://github.com/Pokibot-org/pokirobot_soft west-workspace
cd west-workspace/pokirobot_soft

# pull Zephyr upstream repository and modules (may take a while)
west update
```

cd west-workspace/pokirobot_soft

```sh
west-workspace/                     # contains .west/config
│
├── pokirobot_soft/                 # application firmware repository
│   ├── app/                        # application source files
│   ├── boards/                     # board specifications
│   ├── drivers/                    # additional drivers
│   ├── dts/                        # additional dts bindings
│   ├── tests/                      # unit test source files
│   └── west.yml                    # main manifest file
│
├── modules/                        # modules imported by Zephyr and CC firmware
|
├── tools/                          # tools used by Zephyr
│
└── zephyr/                         # upstream Zephyr repository
```

If you already have a west workspace set up, you can also re-use it to avoid having many copies of upstream Zephyr and modules:
```sh
# go to your workspace directory
cd your-zephyr-workspace

# pull the firmware
git clone https://github.com/Pokibot-org/pokirobot_soft

# re-configure and update the workspace
# (to be done each time you switch between applications in same workspace)
west config manifest.path pokirobot_soft
west update
```
# BUILD

Inside the app directory (pokirobot_soft):
```sh
west build -b nucleo_f446re app -- -DOVERLAY_CONFIG=debug.conf
```

Or in release mode:
```sh
west build -b nucleo_f446re app
```

## Sanity checks

```bash
./tools/check_eol.sh $(find src -name "*.c" -o -name "*.cpp" -o -name "*.h")
```
