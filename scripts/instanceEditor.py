#!/usr/bin/ python
'''
python3 script
python script code for
1. read a sipp task file and only keep the first agent

Author: Tianyi Gu
Date: 11/29/2021
'''

__author__ = 'TianyiGu'

import argparse
import os
# import json
# from subprocess import Popen, PIPE, TimeoutExpired
# import re
# from shutil import copy2
# from operator import getitem

researchHome = "/home/aifs1/gu/phd/research/workingPaper"


def parseArugments():

    parser = argparse.ArgumentParser(description='instanceEdior')

    parser.add_argument(
        '-m',
        action='store',
        dest='maps',
        help='map name: den520d(default)',
        default='den520d')

    parser.add_argument(
        '-t',
        action='store',
        type=int,
        dest='totalInstances',
        help='total instances for the map: 1000(default)',
        default=1000)

    # parser.add_argument(
        # '-s',
        # action='store',
        # dest='subdomain',
        # help='subdomain: tile: uniform, heavy(default), inverse, reverse, sqrt; \
        # pancake: regular, heavy; \
        # racetrack : barto-big,uniform-small, barto-bigger, hanse-bigger-double\
        # vacuumworld: uniform, heavy;',
        # default='heavy')

    # parser.add_argument('-z',
                        # action='store',
                        # dest='size',
                        # help='domain size (default: 4)',
                        # default='4')

    return parser

def main():
    parser = parseArugments()
    args = parser.parse_args()
    print(args)

    fileDir = researchHome+\
        "/situatedSIPP/situatedSIPPCodeBase/instances/singleagent-icaps2020/" +\
        args.maps + "/"

    if not os.path.exists(fileDir):
        print("map",fileDir,"does not exist!")
        return

    print("processing ", args.maps)
    for fileName in os.listdir(fileDir):

        # print("processing ", fileName)
        if fileName[-8:] != "task.xml":
            continue

        print("working on", fileName)

        lines = []
        with open(fileDir+fileName, "r") as f:
            lines = f.readlines()

        if len(lines) != args.totalInstances + 6:
            print("sikp ", fileName)
            continue

        lines = lines[:4] + lines[args.totalInstances + 4:]
        lines[2] = lines[2].replace(str(args.totalInstances+1), '1')

        with open(fileDir+fileName, "w") as f:
            f.writelines(lines)


if __name__ == '__main__':
    main()
