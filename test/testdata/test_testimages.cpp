

#include <gtest/gtest.h>

#include "testimages.h"

TEST( TestImages, decodeToBytes )
{
  for( int i = 0; i <  NUM_TEST_IMAGES; ++i ) {
    std::vector<BYTE> out = TestImage(i);
    ASSERT_EQ( out.size(), TestImageWidth*TestImageHeight );
  }

}
