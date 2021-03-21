#!/bin/sh
exec socat \
    PTY,link="$1",rawer,wait-slave \
    EXEC:"rosrun stockfish stockfish",pty,setsid,ctty,echo=0
