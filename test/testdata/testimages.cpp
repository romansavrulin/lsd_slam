#include "testimages.h"

#include "base64_impl.h"

std::vector<BYTE> TestImage( int i ) {
  std::vector<BYTE> out = base64_decode( TestImages[i] );
  assert( out.size() == TestImageWidth*TestImageHeight );
  return out;
}
