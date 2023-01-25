#!/usr/bin/env python3

# general import
import can
import getopt
import os
import sys
import time

class CanSendTest:
    __usage = f"""Usage:{sys.argv[0]} <ONLY ONE OPTION>
Options:
    -h --help       Print this help message and exit.
    -i --inverter   Send can signal to test inverter.
    -o --once       Only send test can signal once.
    -r --repeat     Repeatedly send test can signal."""

    __option = "hior"

    __long_option = [
        "help",
        "inverter",
        "once",
        "repeat"
    ]

    __bus = None

    # internal states
    __can_activated = False

    def __del__(self):
        if self.__can_activated:
            os.system("sudo ifconfig can0 down")

    def run(self):
        try:
            opts, args = getopt.getopt(sys.argv[1:], self.__option, self.__long_option)

        except getopt.GetoptError:
            print(self.__usage, file=sys.stderr)
            sys.exit(1)

        if len(opts) != 1:
            print(self.__usage, file=sys.stderr)
            sys.exit(1)

        for opt, val in opts:
            if opt in ("-h", "--help"):
                print(self.__usage, file=sys.stderr)
                sys.exit(1)

            elif opt in ("-i", "--inverter"):
                self.__test_inverter()

            elif opt in ("-o", "--once"):
                self.__send_once()

            elif opt in ("-r", "--repeat"):
                self.__send_repeatedly()

    def __configure_can(self):
        if not self.__can_activated:
            os.system("sudo ip link set can0 type can bitrate 100000")
            os.system("sudo ifconfig can0 up")
            self.__bus = can.interface.Bus(channel="can0", bustype="socketcan")

        self.__can_activated = True

    def __send_once(self):
        self.__configure_can()
        msg = can.Message(arbitration_id=0x010, is_extended_id=False, data=[0, 1, 2, 3, 4, 5, 6, 7])
        self.__bus.send(msg)
        print(f"Send signal: {msg}.")

    def __send_repeatedly(self):
        self.__configure_can()
        while True:
            self.__send_once()
            time.sleep(0.5)

    def __test_inverter(self):
        self.__configure_can()
        print("Not implemented yet.", file=sys.stderr)

if __name__ == "__main__":
    can_send_test = CanSendTest()
    
    try:
        can_send_test.run()

    except KeyboardInterrupt:
        print("Terminated by \"Ctrl + C\".")

    finally:
        print("Can send test finished.")
