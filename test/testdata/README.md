
# To make base64 encoded images

convert 00001.png -depth 8  gray:- | base64 > 00001.base64
