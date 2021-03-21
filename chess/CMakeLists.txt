cmake_minimum_required(VERSION 3.10)
project(chess)

find_package(
    catkin REQUIRED
    COMPONENTS
        message_generation
        rospy
)

add_message_files(
    FILES
        Chessboard.msg
        Chesspiece.msg
        Move.msg
        Player.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS
        rospy
        std_msgs
        stockfish
)
