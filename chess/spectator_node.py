#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import functools

import rospy

from chess.msg import Chessboard, Chesspiece, Player


captures = {
    Player.WHITE: [],
    Player.BLACK: [],
}


def handle_capture(player, capture):
    captures[player].append(capture)


def prettyprint(board):
    symbols = {
        Player.WHITE: dict(zip('prnbqk', u'♙♖♘♗♕♔')),
        Player.BLACK: dict(zip('prnbqk', u'♟♜♞♝♛♚')),
    }

    for row in xrange(7, -1, -1):  # draw row 1 last, at the bottom
        line = ''
        for col in range(8):
            piece = board.squares[8*row+col]
            line += '\x1b[{}m'.format([47,107][(row + col) % 2])
            line += ' ' + symbols[piece.player.id].get(piece.kind, ' ') + ' '
            line += '\x1b[0m'

        if row == 7:
            line += '  ' + ''.join(symbols[Player.WHITE][c.kind]
                                   for c in captures[Player.BLACK])
        if row == 0:
            line += '  ' + ' '.join(symbols[Player.BLACK][c.kind]
                                    for c in captures[Player.WHITE])
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
