#!/usr/bin/env python3

from string import Template
import subprocess
import base64
from scipy import misc

header_template = """#include "testimages.h"

const std::array<std::string,NUM_TEST_IMAGES> TestImages = {
"""

images = [ "00001.png", "00002.png", "00003.png" ]


header = Template( header_template )


with open("testimages_data.cpp", 'w') as f:
    f.write( header.substitute() )

    for i in images:
        img = misc.imread(i)
        f.write( "\"%s\"" % base64.b64encode(img).decode('ascii') )

        if i != images[-1]:
            f.write(",\n")



    f.write(" };\n")
