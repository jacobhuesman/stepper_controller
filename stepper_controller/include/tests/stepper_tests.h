#include <test.h>
#include <stepper.h>

TEST_SUITE(Stepper Tests)

TEST(convertAngle uint16_t->float)
{
  ASSERT_FEQUAL(Stepper::convertAngle((uint16_t)4096),  0.0f, 1e-6f);
  ASSERT_FEQUAL(Stepper::convertAngle((uint16_t)   0), -0.5f, 1e-6f);
  ASSERT_FEQUAL(Stepper::convertAngle((uint16_t)8192),  0.5f, 1e-6f);
}

TEST(convertAngle float->uint16_t)
{
  ASSERT_EQUAL(Stepper::convertAngle( 0.0f), 4096);
  ASSERT_EQUAL(Stepper::convertAngle(-0.5f),    0);
  ASSERT_EQUAL(Stepper::convertAngle( 0.5f), 8192);
}
