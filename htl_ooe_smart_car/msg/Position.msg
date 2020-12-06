# the position message is used when configuring drive mode to
# assign a  servo to a specific wheel positon in the drive system
# postions are specific toe the desired drive mode
# ackerman has only one position
# 1 = drive
# differential has eight positons
# 1 = POSITION_LEFTFRONT_PHASE_A
# 2 = POSITION_LEFTFRONT_PHASE_B
# 3 = POSITION_RIGHTFRONT_PHASE_A
# 4 = POSITION_RIGHTFRONT_PHASE_B
# 5 = POSITION_LEFTREAR_PHASE_A
# 6 = POSITION_LEFTREAR_PHASE_B
# 7 = POSITION_RIGHTREAR_PHASE_A
# 8 = POSITION_RIGHTREAR_PHASE_B
# mecanum has four positions
# 1 = POSITION_LEFTFRONT_PHASE_A
# 3 = POSITION_RIGHTFRONT_PHASE_A
# 5 = POSITION_LEFTREAR_PHASE_A
# 7 = POSITION_RIGHTREAR_PHASE_A
# multiple servos/motors may be used for each positon

int32 servo
int32 position
