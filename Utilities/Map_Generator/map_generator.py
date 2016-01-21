#!/usr/bin/python
"""Converts the desired bitmap into a string usable as an array in a C program.

Example:
    Run the following command to generate a text file containing an array that is
    ready to be copied and pasted into a C (or C like) program.

    $ python map_generator.py -i <image name>.bmp
"""

import sys
from PIL import Image

__author__ = 'Alex Bennett'
__version__ = '1.0'

try:
    if sys.argv[1] == '-i':
        # Import the image
        img = Image.open(sys.argv[2])

        # Validate the formats
        if img.format != 'BMP':
            print 'Incorrect image format, pleast select a .bmp file'

        print 'Converting bitmap...'

        # Parse the image
        output = []
        w, h = img.size
        for i in range(w):
            row = []
            for j in range(h):
                if img.getpixel((j, i)) == 0:
                        row.append('\'x\'')
                else:
                        row.append('\'o\'')
            output.append(row)

        # Write the output file
        with open(sys.argv[2].replace('.bmp', '') + '.txt', 'w') as f:
            f.write('const char output[' + str(h) + '][' + str(w) + '] = {')
            for i in range(len(output)):
                if i == len(output) - 1:
                    f.write('{' + ', '.join(output[i]) + '}')
                else:
                    f.write('{' + ', '.join(output[i]) + '}, \n')
            f.write('}')

        print 'Bitmap converted to array, check ' + sys.argv[2].replace('.bmp', '') + '.txt'
    else:
        print 'Please select an image. Usage: map_generator.py -i <image name>.bmp'
except Exception:
    print 'Usage: map_generator.py -i <image name>.bmp'
