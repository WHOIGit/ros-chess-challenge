# ROS Chess

**Objective:** Implement a ROS node to compete two chess engines (AIs) against one another.

A chess engine is provided to you. You will write a single "driver" node but two copies will be executed, each talking to its own instance of the chess engine.

The driver should connect to the chess engine "serial device" and initialize the game (see below).

The node representing the white player goes first, asking its engine to make a move. This move is published to the `/white/move` topic as a `chess/Move` message.

If the player captured a piece, a `chess/Chesspiece` message is published to `/white/capture`. 

The new state of the chessboard (a `chess/Chessboard` message) is published to `/chessboard`.

Finally, a `std_msgs/Empty` message is published to `/white/done` to signal the black player's turn to move.

The black player's node works identically, using the `/black` prefix instead.

You do not need to detect check or checkmate, prevent the other player from sending invalid moves, etc.


## Stockfish

The chess engine used in this project is [Stockfish][]. It is wrapped up in a ROS package called `stockfish`.

  [stockfish]: https://stockfishchess.org

For added flavor, you will be communicate with Stockfish over a pseudoterminal (or PTY), which acts like a serial port device.

Once your ROS workspace is set up, you can use the provided launch file to start Stockfish with a "serial device" at `/tmp/stockfish`:

    roslaunch stockfish stockfish.launch device:=/tmp/stockfish

In another window, you can connect and interact with Stockfish:

    picocom --echo /tmp/stockfish
    # press Ctrl-a, Ctrl-x to exit


## Universal Chess Interface

Stockfish uses the [Universal Chess Interface (UCI)][uci], a line-based textual protocol. As the protocol is somewhat esoteric, we will focus on a subset of messages, ignoring the rest.

  [uci]: https://www.shredderchess.com/chess-features/uci-universal-chess-interface.html

Before writing any code, it is recommended that you use `picocom` to familiarize yourself with the protocol.

To set up the game:

  1. Upon starting, the engine identifies itself as Stockfish.

  2. The driver sends `uci` to enable the UCI protocol. The engine sends back some information (ignore it) followed by `uciok`.

  3. The driver begins a new game with `ucinewgame`.

  4. The driver inquires if the engine is ready with `isready`, and the engine replies `readyok`.

To ask the engine to make a move:

  1. The driver sends `position startpos moves` followed by a space-delimited list of the moves that have taken place so far, if any. There is no response.

  2. The driver asks the engine to make a move. For instance, `go movetime 1000` limits the engine to find a move in one second. The engine replies with (for example) `bestmove c7c5` (anything else can be ignored).

Moves are represented by a pair of coordinates; `g1f3` conveys the movement of a piece from column G, row 1 to column F, row 3. There are two special cases:

1. When a pawn reaches the opposite side of the board, it may be [promoted][promotion] to another kind of piece, usually a queen. In this case, the move is represented as above with an additional character representing the new kind, **q** for queen, **r** for rook, **n** for knight, **b** for bishop, as in: `c2c3q`.

2. When [castling][], the king moves two positions towards a rook, e.g., `e1g1`, and the rook "jumps" to the square the king crossed (`h1f1`). *Only* the king's move is expressed to the engine.

  [castling]: https://en.wikipedia.org/wiki/Castling
  [promotion]: https://en.wikipedia.org/wiki/Promotion_(chess)


## Technical Requirements

This project was developed against ROS Melodic and Ubuntu 18.04, but you may opt to use ROS Noetic and Ubuntu 20.04 at your preference. (Please contact us if you encounter any difficulty.)

You may write your solution in C++ or Python. If you choose to implement in Python on ROS Noetic, use Python 3.

You should test your solution with `roslaunch chess chess.launch`. You may modify `chess.launch` as necessary to start your nodes.

You may need to install additional tools:

    sudo apt install -y \
        picocom \
        python-catkin-tools \
        socat

The Stockfish sources are included as a Git submodule, which must be pulled with:

    git submodule update --init --recursive

In Python, you can use [pySerial][] for interfacing with the "serial port".

  [pySerial]: https://pyserial.readthedocs.io/en/latest/pyserial.html


## Tips

You can reach out for clarification on any point, or if you get stuck and need a hint. We aren't expecting to find candidates who never get stuck.

Be mindful that your code will be reviewed to evaluate your proficiency with ROS and your chosen language. Favor clarity of intent over brevity.

If Stockfish seems to make an illegal move, it is likely that you have a bug in your model of the game state.

A solution in Python is possible in under 250 lines. This is not a limit.
