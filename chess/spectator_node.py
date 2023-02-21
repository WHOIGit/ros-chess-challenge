#!/usr/bin/env python3
import rospy

from chess.msg import Chessboard, Chesspiece, Player


captures = {
    Player.WHITE: [],
    Player.BLACK: [],
}


def handle_capture(player, capture):
    captures[player].append(capture)


def prettyprint(board):
    symbols = dict(zip('prnbqk', u'♟♜♞♝♛♚'))

    # If these don't look good on your terminal, you can adjust them
    # https://en.wikipedia.org/wiki/ANSI_escape_code#Colors
    ansi_fg = { Player.WHITE: "38;5;240", Player.BLACK: "38;5;232" }
    ansi_bg = [ "48;5;250", "48;5;255" ]  # black, white

    for row in xrange(7, -1, -1):  # draw row 1 last, at the bottom
        line = ''
        for col in range(8):
            piece = board.squares[8*row+col]
            line += u'\x1b[{};{}m {} '.format(
                ansi_fg[piece.player.id],
                ansi_bg[(row + col) % 2],
                symbols.get(piece.kind, ' '),
            )

        if row == 7:
            line += '\x1b[0;{}m  '.format(ansi_fg[Player.WHITE])
            line += ''.join(symbols[c.kind] for c in captures[Player.BLACK])
        elif row == 0:
            line += '\x1b[0;{}m  '.format(ansi_fg[Player.BLACK])
            line += ''.join(symbols[c.kind] for c in captures[Player.WHITE])
        
        line += '\x1b[0m'
        print(line)
    print()


def main():
    rospy.init_node('spectator')
    rospy.Subscriber('/white/capture', Chesspiece,
        functools.partial(handle_capture, Player.WHITE))
    rospy.Subscriber('/black/capture', Chesspiece,
        functools.partial(handle_capture, Player.BLACK))
    rospy.Subscriber('/chessboard', Chessboard, prettyprint)
    rospy.spin()


if __name__ == '__main__':
    main()
