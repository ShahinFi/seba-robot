"""Command-line helper for creating local web password hashes."""

import getpass
import sys

from .auth import hash_password


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--stdin":
        password = sys.stdin.read()
    else:
        password = getpass.getpass("Password: ")

    print(hash_password(password))


if __name__ == "__main__":
    main()
