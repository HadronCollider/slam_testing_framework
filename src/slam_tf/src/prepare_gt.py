#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from tf.transformations import quaternion_from_euler

def create_gt(path_to_file, path_to_save, parse_fun):
    with open(path_to_file, 'r') as input:
        with open(path_to_save, 'w') as output:	
            for line in input:
                if '#' in line:
                    continue
                output.write("{}\n".format(parse_fun(line)))
                
def get_parser(dataset_type):
    if dataset_type == 'mit':
        return parser_mit
    elif dataset_type == 'frog':
        return parser_frog
    else:
        raise ValueError('Unexpected DATASET_TYPE={}'.format(dataset_type))

def parser_mit(string):
    """
    example: "1328208249.001690,29.9621,130.791,0.406198"
    """
    elems = string.split(',')
    elems[0]="{}.{}".format(elems[0][0:10], elems[0][10:])
    angle = quaternion_from_euler(0, 0, float(elems[3]))
    return "{} {} {} 0.0 {:f} {:f} {:f} {:f}".format(elems[0], elems[1], elems[2], angle[0], angle[1], angle[2], angle[3])

def parser_frog(string):
    """
    example: "1411471140.795906296, 15.8891, 119.224, 0.0217176"
    """
    elems = string.split(', ')
    angle = quaternion_from_euler(0, 0, float(elems[3]))
    return "{} {} {} 0.0 {:f} {:f} {:f} {:f}".format(elems[0], elems[1], elems[2], angle[0], angle[1], angle[2], angle[3])

if __name__ == "__main__":
    len_arg = len(sys.argv)
    if len_arg == 3:
        dataset_type = sys.argv[1]
        path_to_file = sys.argv[2]
        path_to_save = path_to_file + '.gt'
        create_gt(path_to_file, path_to_save, get_parser(dataset_type))
    elif len_arg == 4:
        DATASET_TYPE = sys.argv[1]
        path_to_file = sys.argv[12]
        path_to_save = sys.argv[3]
        create_gt(path_to_file, path_to_save, get_parser(dataset_type))
    else:
        print('Not enough args. Usage: python prepare.py <dataset_type> path_to_file <path_to_save>')
