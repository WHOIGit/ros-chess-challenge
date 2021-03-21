cmake_minimum_required(VERSION 3.0.2)
project(stockfish)

find_package(catkin REQUIRED)
catkin_package()

# The Stockfish project uses a Makefile; just invoke it.
add_custom_target(
    stockfish ALL
    COMMAND make -j build ARCH=x86-64-modern
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Stockfish/src
)
