import serial
import commands as cmd
import time as t


SERIAL_COMMUNICATION_UNKNOWN_RESPONSE_ERROR_MSG = "SerialCommunicationUnknownResponse: The received serial response was not valid."


class SerialCommunicationUnknownResponse(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    SerialCommunicationUnknownResponse: The received serial response was not valid.
    """
    pass


class CommunicationProtocol:
    def __init__(self, com, baudrate, timeout):
        self.com = com
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection = serial.Serial(port=self.com, baudrate=self.baudrate, timeout=self.timeout)

    def send_command(self, command):
        self.connection.write(bytes(command, 'utf-8'))

        while True:
            resp = str(self.connection.readline())
            print(resp)

            if resp == cmd.RESPONSE:
                break

            if resp != cmd.EMPTY:
                raise SerialCommunicationUnknownResponse(SERIAL_COMMUNICATION_UNKNOWN_RESPONSE_ERROR_MSG)


if __name__ == "__main__":
    com_test = CommunicationProtocol("COM3", 9600, 0.1)
    t.sleep(5)
    com_test.send_command(cmd.HOME_PRESSER)
    com_test.send_command(cmd.HOME_GRIPPER)
    com_test.send_command(cmd.CLOSE_GRIPPER)
    t.sleep(5)
    com_test.send_command(cmd.PREPARE_PRESSING)
    com_test.send_command(cmd.PRESS_ALLERGEN + "2500")

