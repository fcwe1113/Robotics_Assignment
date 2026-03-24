#!/usr/bin/env python3

if __name__=="__main__":
    while True:
        status = f"""
        current mode: {}
        """
        options = """
        1. do something
        4. exit
        """
        inputt = input(status, options)
        if inputt == 1:
            ...
        elif inputt == 4:
            print("exiting...")
            break