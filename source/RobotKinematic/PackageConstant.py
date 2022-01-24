import pathlib

class Const:

    ROOT                 = pathlib.Path(__file__).parent.resolve()
    ICON_PATH            = pathlib.Path(ROOT,      "Icons")
    ICON_ADD_MODE        = pathlib.Path(ICON_PATH, "add_mode.png")
    ICON_GRAB_MODE_OPEN  = pathlib.Path(ICON_PATH, "grab_mode_open.png")
    ICON_GRAB_MODE_CLOSE = pathlib.Path(ICON_PATH, "grab_mode_close.png")
    ICON_LOCK_OPEN       = pathlib.Path(ICON_PATH, "lock_open.png")
    ICON_LOCK_CLOSE      = pathlib.Path(ICON_PATH, "lock_close.png")
    ICON_PLAY            = pathlib.Path(ICON_PATH, "play.png")
    ICON_STOP            = pathlib.Path(ICON_PATH, "stop.png")
