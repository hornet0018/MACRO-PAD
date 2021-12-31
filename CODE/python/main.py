import board

from kmk.kmk_keyboard import KMKKeyboard
from kmk.keys import KC
from kmk.matrix import DiodeOrientation
from kmk.modules.encoder import EncoderHandler
from kmk.modules.layers import Layers

layers = Layers()
keyboard = KMKKeyboard()
encoder_handler = EncoderHandler()
keyboard.modules = [layers, encoder_handler]

keyboard.row_pins = (board.GP10, board.GP11)
keyboard.col_pins = (board.GP12, board.GP13, board.GP14, board.GP15)
keyboard.diode_orientation = DiodeOrientation.COLUMNS
encoder_handler.pins = ((board.GP7, board.GP8, None, False),)

keyboard.debug_enabled = False

keyboard.keymap = [
    [  # Base
        KC.N1, KC.N2, KC.N3, KC.N4,
        KC.N5, KC.N6, KC.N7
    ]
]

encoder_handler.map = [((KC.VOLU, KC.VOLD),)]

if __name__ == '__main__':
    keyboard.go()
