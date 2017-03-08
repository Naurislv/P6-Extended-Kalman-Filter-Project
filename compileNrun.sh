#!/bin/bash

cpp_file="$1"
g++ -o output_file ${cpp_file}
output_file
rm output_file
