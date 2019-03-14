#include <test.h>

TEST_SUITE(Sanity Checks)

TEST(ASSERT_FEQUAL)
{
  ASSERT_FEQUAL(1.0, 1.0,    1e-6);
  ASSERT_FEQUAL(1.0, 1.0001, 1e-3);
  //ASSERT_FEQUAL(1.0, 1.1,    1e-3);
}
