from serial_comm import SerialComm


def pee(motorPWM: int, ser_write_command: SerialComm):
    command_str = f"<{motorPWM}>"
    ser_write_command.write_line(command_str)
