uint8 src_row  # 1 to 8
uint8 src_col  # 1 (a) to 8 (h)
uint8 dst_row  # 1 to 8
uint8 dst_col  # 1 (a) to 8 (h)

# See: https://en.wikipedia.org/wiki/Castling
bool is_castling

# Note: The piece may only change if a promotion occurs
# See: https://en.wikipedia.org/wiki/Promotion_(chess)
bool is_promotion
chess/Chesspiece piece
