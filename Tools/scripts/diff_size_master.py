#!/usr/bin/env python
import os
import json
from argparse import ArgumentParser

header_text = {
    'target': 'Target',
    'binary_path': 'Binary',
    'size_text': 'Text',
    'size_data': 'Data',
    'size_bss': 'BSS',
    'size_total': 'Total',
}

parser = ArgumentParser(description="Print binary size difference with master.")
parser.add_argument("-b", "--board", dest='board', help="Board type to use")
parser.add_argument("-m", "--master", dest='master_path', help="Master binary reference")

args = parser.parse_args()


def sizes_for_file(filepath):
    cmd = "size %s" % (filepath,)
    stuff = os.popen(cmd).read()
    lines = stuff.splitlines()[1:]
    l = []
    for line in lines:
        row = line.strip().split()
        l.append(dict(
            size_text=int(row[0]),
            size_data=int(row[1]),
            size_bss=int(row[2]),
            size_total=int(row[3]),
        ))
    return l


def print_table(summary_data_list, summary_data_list_master, header):
    max_widths = []
    table = [[] for _ in range(len(summary_data_list))]

    header_row = []
    for h in header:
        txt = header_text.get(h, h)
        header_row.append(txt)
        max_width = len(txt)
        for i, row_data in enumerate(summary_data_list):
            if summary_data_list[i]["target"] == summary_data_list[i]["target"]:
                
                txt = str(row_data.get(h, '-'))
                master_data = summary_data_list_master[i][h]
                diff_master = (master_data - int(txt)) * 100 / master_data
                txt = txt + " (%.2f%%)" % diff_master
                table[i].append(txt)
            else:
                print("Summary don't match, exiting.")
                exit(1)

            w = len(txt)
            if w > max_width:
                max_width = w
        max_widths.append(max_width)

    sep = '  '
    fmts = ['{:<%d}' % w for w in max_widths]
    header_row = sep.join(fmts).format(*header_row)
    print(header_row)

    line = ('-' * len(sep)).join('-' * w for w in max_widths)
    print(line)

    for row in table:
        fmts = []
        for j, v in enumerate(row):
            w = max_widths[j]
            try:
                float(v)
            except ValueError:
                fmts.append('{:<%d}' % w)
            else:
                fmts.append('{:>%d}' % w)
        row = sep.join(fmts).format(*row)
        print(row)