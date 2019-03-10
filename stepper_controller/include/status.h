/*******************************************************************************
 * @file    status.h
 * @brief   Common macros and functions for indicating status
 * @author  Jacob Huesman
 *  ******************************************************************************/

// Print Macros (Can handle up to ten arguments)
#define GET_PRINT_VERSION(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, NAME, ...) NAME
#define PRINT(HEADER, ...) GET_PRINT_VERSION(__VA_ARGS__, PRINTN, PRINTN, PRINTN, \
                                    PRINTN, PRINTN, PRINTN, \
                                    PRINTN, PRINTN, PRINTN, \
                                    PRINT1)(HEADER, __VA_ARGS__)

#if defined(DEBUG_PRINTING) && (DEBUG_PRINTING == 1)

#define INIT_DEBUG() BOARD_InitDebugConsole()

#define PRINT1(HEADER, FORMAT)      printf(HEADER FORMAT "\n")
#define PRINTN(HEADER, FORMAT, ...) printf(HEADER FORMAT "\n", __VA_ARGS__)
#define SPACE(...) printf("\n")

#else

// Define but do nothing
#define INIT_DEBUG()
#define PRINT1(HEADER, FORMAT)
#define PRINTN(HEADER, FORMAT, ...)
#define SPACE(...)

// Pass any function call through
#define STATUS(FNCALL) { FNCALL; }

#endif

#define INFO(...)  PRINT("[INFO] ",    __VA_ARGS__);
#define WARN(...)  PRINT("[WARNING] ", __VA_ARGS__);
#define ERROR(...) PRINT("[ERROR] ",   __VA_ARGS__);


#ifndef STATUS
#define STATUS(FNCALL)                       \
{                                            \
  if (FNCALL == kStatus_Success)           \
  {                                        \
    INFO("Call to "#FNCALL" succeeded"); \
  }                                        \
  else                                     \
  {                                        \
    ERROR("Call to "#FNCALL" failed");   \
  }                                        \
}
#endif
