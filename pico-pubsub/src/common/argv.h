#ifndef H_argv_h
#define H_argv_h
// #include "Arduino.h"

#define CLI_BUFFER_MAX 100 // the maximum length of a command line
#define CLI_TOKENS_MAX 10  // the max number of tokens - max value of argc


/**
 * This is the equivalent of argc argc. Its a list of tokens
 * derived from an input cimmand line
 */
struct Argv
{
    char* token_at(int i);
    void  copyTo(Argv& other);
    void  dump(const char* msg);
    int   token_count;
    char* token_positions[CLI_TOKENS_MAX];
    int   token_lengths[CLI_TOKENS_MAX];
    /**
     * A copy of the original input line - is modified during the tokenization process
     * this class has its own copy to make it standalone
     */
    char  m_line_buffer[CLI_BUFFER_MAX];
};
void tokenize_line(char* line, Argv& argv);

#endif