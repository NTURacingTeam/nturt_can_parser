#!/usr/bin/env python3

# general import
import can
import getopt
import os
import sys

class CanReceiveTest:
    __usage = f"""Usage:{sys.argv[0]} <ONLY ONE OPTION>
Options:
    -h --help       Print this help message and exit.
    -o --once       Only receive can signal once.
    -r --repeat     Repeatedly receive can signal."""

    __option = "hor"

    __long_option = [
        "help",
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

            elif opt in ("-o", "--once"):
                self.__receive_once()

            elif opt in ("-r", "--repeat"):
                self.__receive_repeatedly()

    def __configure_can(self):
        if not self.__can_activated:
            os.system("sudo ip link set can0 type can bitrate 100000")
            os.system("sudo ifconfig can0 up")
            self.__bus = can.interface.Bus(channel="can0", bustype="socketcan")

        self.__can_activated = True

    def __receive_once(self):
        self.__configure_can()
        msg = self.__bus.recv(10)
        if msg is None:
            print("Timeout occurred, no message after 10 seconds.", file=sys.stderr)
            sys.exit(1)
        
        print(f"Received signal: {msg}.")

    def __receive_repeatedly(self):
        self.__configure_can()
        while True:
            self.__receive_once()

if __name__ == "__main__":
    can_receive_test = CanReceiveTest()
    
    try:
        can_receive_test.run()

    except KeyboardInterrupt:
        print("Terminated by \"Ctrl + C\".")

    finally:
        print("Can receive test finished.")
