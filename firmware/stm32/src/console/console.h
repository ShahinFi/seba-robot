#ifndef CONSOLE_H
#define CONSOLE_H

/*
 * Initializes command-line buffering and framed command
 * protocol state.
 */
void Console_Init(void);

/*
 * Polls buffered serial input and executes complete command
 * lines. Call regularly from the main loop.
 */
void Console_Process(void);

#endif
