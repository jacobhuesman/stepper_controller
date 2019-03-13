#ifndef EVAL_H
#define EVAL_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

class Test
{
public:
  Test(void (*test)(bool*), std::string name);
  Test(std::string suite);
  bool run();
  static bool runAll();

  void (*test)(bool*);
  std::string name;
  std::string suite;
  static std::string* current_suite;

private:
  bool pass;
  static uint32_t tests_size, suites_size;
  static std::string* suites[];
  static Test* tests[];
};

#define CAT2(x,y) x##y
#define TEST_SUITE(name) _TEST_SUITE(name, __COUNTER__)
#define _TEST_SUITE(name, counter)  \
  Test CAT2(suite_, counter) (#name);

#define CAT3(x,y,z) x##y##z
#define TEST(name) _TEST1(name, __COUNTER__)
#define _TEST1(name, counter) \
  _TEST2(name, CAT3(test_, counter, _fn), CAT2(test_, counter))
#define _TEST2(name, function, object)  \
  void function(bool *state);           \
  Test object(&function, #name);        \
  void function(bool *state)

// Helper macros
#define STRING_LENGTH(s) (sizeof(s) - 1)

#define INT_LENGTH(value)          \
({                                 \
    int divisor = 10, length = 1;  \
    while ((value / divisor) > 0)  \
    {                              \
        length++;                  \
        divisor = divisor * 10;    \
    }                              \
  length;                        \
})

#define CLONE_ARRAY(n, original, copy) \
{                                      \
    copy = malloc(n*4);                \
    for (int i = 0; i < n; i++)        \
    {                                  \
        copy[i] = original[i];         \
    }                                  \
}

#define MAX(a, b) ((a > b) ? a : b)

// Unit testing framework

#define PASS(FORMAT, ...) printf("    [PASS] " FORMAT, ##__VA_ARGS__);
#define FAIL(FORMAT, ...) printf("    [FAIL] " FORMAT, ##__VA_ARGS__);
#define ASSERT(ASSERTION)       \
({                              \
    bool pass = false;          \
    if (ASSERTION)              \
    {                           \
        PASS(#ASSERTION "\n");  \
        pass = true;            \
    }                           \
    else                        \
    {                           \
        FAIL(#ASSERTION "\n");  \
    }                           \
    *state &= pass;             \
    pass;                       \
})

#define ASSERT_ARRAY_EQ(n,A,B)       \
{                                    \
    int pass = 1;                    \
    for (int i = 0; i < n; i++)      \
    {                                \
        if (A[i] != B[i])            \
        {                            \
            pass = 0;                \
            FAIL(#A "[%i] = %i != "  \
           #B "[%i] = %i\n",   \
         i, A[i], i, B[i]);  \
            break;                   \
        }                            \
    }                                \
    if (pass)                        \
    {                                \
        PASS(#A " == " #B "\n");     \
    }                                \
}

#define ASSERT_SQUARE_MATRIX_EQ(n,A,B)               \
{                                                    \
    int pass = 1;                                    \
    for (int i = 0; i < n; i++)                      \
    {                                                \
        for (int j = 0; j < n; j++)                  \
        {                                            \
            if (A[i][j] != B[i][j])                  \
            {                                        \
                pass = 0;                            \
                FAIL(#A "[%i][%i] = %f,!= "          \
             #B "[%i][%i] = %f\n",           \
           i, j, A[i][j], i, j, B[i][j]);  \
                break;                               \
            }                                        \
        }                                            \
        if (!pass)                                   \
        {                                            \
            break;                                   \
        }                                            \
    }                                                \
    if (pass)                                        \
    {                                                \
        PASS(#A " == " #B "\n");                     \
    }                                                \
}

#define ASSERT_SQUARE_MATRIX_DLEQ(n,A,B,D)           \
{                                                    \
    int pass = 1;                                    \
    for (int i = 0; i < n; i++)                      \
    {                                                \
        for (int j = 0; j < n; j++)                  \
        {                                            \
            if ((A[i][j] - B[i][j]) > D)             \
            {                                        \
                pass = 0;                            \
                FAIL(#A "[%i][%i] = %f,!= "          \
           #B "[%i][%i] = %f\n",           \
           i, j, A[i][j], i, j, B[i][j]);  \
                break;                               \
            }                                        \
        }                                            \
        if (!pass)                                   \
        {                                            \
            break;                                   \
        }                                            \
    }                                                \
    if (pass)                                        \
    {                                                \
        PASS(#A " == " #B "\n");                     \
    }                                                \
}

#define STR(x) #x
#define PRINT_ARRAY(n,A,padding)              \
{                                             \
    printf("%s = {", #A);                     \
    printf("%" STR(padding) "i", A[0]);       \
    for (int i = 1; i < n; i++)               \
    {                                         \
        printf(",%" STR(padding) "i", A[i]);  \
    }                                         \
    printf(" }\n");                           \
}

// PRINT_ARRAYS
// ---------------------
// | array 1 | array 2 |
// ---------------------
// |      12 |      12 |

#define PRINT_ARRAYS(n,A,B)                                         \
{                                                                   \
    /* find maximum character size   */                             \
    int max_size = MAX(sizeof(#A) - 1, sizeof(#B) - 1);             \
    for (int i = 0; i < n; i++)                                     \
    {                                                               \
        max_size = MAX(max_size, INT_LENGTH(A[i]));                 \
        max_size = MAX(max_size, INT_LENGTH(B[i]));                 \
    }                                                               \
    for (int i = 0; i < (max_size * 2 + 7); i++)                    \
    {                                                               \
        putchar('-');                                               \
    }                                                               \
    putchar('\n');                                                  \
    printf("| %*s | %*s |\n", max_size, #A, max_size, #B);          \
    for (int i = 0; i < (max_size * 2 + 7); i++)                    \
    {                                                               \
        putchar('-');                                               \
    }                                                               \
    putchar('\n');                                                  \
    for (int i = 0; i < n; i++)                                     \
    {                                                               \
        printf("| %*i | %*i |\n", max_size, A[i], max_size, B[i]);  \
    }                                                               \
    for (int i = 0; i < (max_size * 2 + 7); i++)                    \
    {                                                               \
        putchar('-');                                               \
    }                                                               \
    putchar('\n');                                                  \
}

// PRINT_SQUARE_MATRIX
// ----------------------------
// | 1.23e3 | 1.23e3 | 1.23e3 |
// ----------------------------
// | 1.23e3 | 1.23e3 | 1.23e3 |
// ----------------------------
// | 1.23e3 | 1.23e3 | 1.23e3 |
// ----------------------------
#define PRINT_SQUARE_MATRIX(n,A,p)            \
{                                             \
    printf("%s:\n", #A);                      \
    int length = (p+8)*n + 1;                 \
    for (int i = 0; i < n; i++)               \
    {                                         \
        for (int j = 0; j < length; j++)      \
        {                                     \
            putchar('-');                     \
        }                                     \
        putchar('\n');                        \
        putchar('|');                         \
        for (int j = 0; j < n; j++)           \
        {                                     \
            printf(" %.*e |", p-1, A[i][j]);  \
        }                                     \
        putchar('\n');                        \
    }                                         \
    for (int k = 0; k < length; k++)          \
    {                                         \
        putchar('-');                         \
    }                                         \
    putchar('\n');                            \
}

#endif // EVAL_H
