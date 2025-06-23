"""
set_gps_entries_nan.py

NOTE: This script **overwrites the files in-place** — do not use if you need to keep the original data.

Usage:
    python set_gps_entries_nan.py <dir> <start> <end>

For each file named <dir>/<number>.txt in the range [start, end], replaces the first two entries
(space-separated values) of the first line with 'NAN'.

Example:
    python set_gps_entries_nan.py ./data 0 99

This will overwrite ./data/0000000000.txt to ./data/0000000099.txt, setting their first two entries to NAN.
"""
import os
import sys

def process_file(filepath):
    with open(filepath, "r") as f:
        line = f.readline().strip()
    values = line.split()
    if len(values) >= 2:
        values[0] = "NAN"
        values[1] = "NAN"
    new_line = " ".join(values)
    with open(filepath, "w") as f:
        f.write(new_line + "\n")

def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <dir> <start> <end>")
        sys.exit(1)
    directory = sys.argv[1]
    start = int(sys.argv[2])
    end = int(sys.argv[3])

    for i in range(start, end + 1):
        filename = f"{i:010d}.txt"
        filepath = os.path.join(directory, filename)
        if os.path.isfile(filepath):
            process_file(filepath)
        else:
            print(f"File not found: {filepath}")

if __name__ == "__main__":
    main()