#include <test.h>

#define TEST_BUFFER 100
#define TEST_SUITE_BUFFER 10
Test* Test::tests[TEST_BUFFER];
uint32_t Test::tests_size = 0;
uint32_t Test::suites_size = 0;
std::string default_suite = "";
std::string *Test::current_suite = &default_suite;
std::string *Test::suites[TEST_SUITE_BUFFER];

Test::Test(void (*test)(bool*), std::string name)
{
  this->pass = true;
  this->test = test;
  this->name = name;
  this->suite = *Test::current_suite;
  Test::tests[Test::tests_size++] = this;
}

Test::Test(std::string suite)
{
  this->suite = suite;
  Test::current_suite = &this->suite;
  bool match = false;
  for (uint32_t i = 0; i < Test::suites_size; i++)
  {
    if (suite == (*suites[i]))
    {
      match = true;
      break;
    }
  }
  if (!match)
  {
    suites[Test::suites_size++] = Test::current_suite;
  }
}

bool Test::run()
{
  printf("  [TEST] %s\n", name.c_str());
  test(&this->pass);
  if (this->pass)
  {
    printf("    [TEST PASSED]\n");
    return true;
  }
  else
  {
    printf("    [TEST FAILED]\n");
    return false;
  }
}

bool Test::runAll()
{
  printf("\n[TEST ALL] Running...\n");
  uint32_t passed = 0;
  for (uint32_t i = 0; i < Test::suites_size; i++)
  {
    printf("[TEST SUITE] %s\n", suites[i]->c_str());
    for (uint32_t j = 0; j < Test::tests_size; j++)
    {
      if (tests[j]->suite == (*suites[i]))
      {
        if (tests[j]->run())
        {
          passed++;
        }
      }
    }
  }

  printf("[TEST SUMMARY] [PASS] %u, [FAIL] %u\n", passed, Test::tests_size - passed);
  if (passed == Test::tests_size)
  {
    return true;
  }
  else
  {
    return false;
  }
}
