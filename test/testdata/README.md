
Rather than load images into the test suite, a set of test images is
compiled in the test suite as binary data.   They are formatted as
raw greyscale images which are base64-encoded as data which is compiled
into the test binary.  

Functions in `testimages.h` can be use to extract and work with test images.

# To make base64 encoded images

convert 00001.png -depth 8  gray:- | base64 > 00001.base64

or use the `make_testdata.py` script which will make the source file `testimages_data.cpp`
