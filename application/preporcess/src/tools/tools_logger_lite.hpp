#pragma once
#include <iostream>
#include <sstream>

/* consider adding boost thread id since we'll want to know whose writting and
 * won't want to repeat it for every single call */

/* consider adding policy class to allow users to redirect logging to specific
 * files via the command line
 */

// USAGE;
// // define and turn off for the rest of the test suite
// loglevel_e loglevel = logERROR;

// void logTest(void) {
//     loglevel_e loglevel_save = loglevel;

//     loglevel = logDEBUG4;

//     log(logINFO) << "foo " << "bar " << "baz";

//     int count = 3;
//     log(logDEBUG) << "A loop with "    << count << " iterations";
//     for (int i = 0; i != count; ++i)
//     {
//         log(logDEBUG1) << "the counter i = " << i;
//         log(logDEBUG2) << "the counter i = " << i;
//     }

//     loglevel = loglevel_save;
// }




enum loglevel_e
    {logFATALERROR,logERROR, logWARNING, logINFO, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};
std::string loglevel_str [] = {
    "\033[1;31mFATAL ERROR\033[0m ",
    "\033[1;31mERROR\033[0m", 
    "\033[1;33mWARN\033[0m ", 
    "\033[1;34mINFO\033[0m ", 
    "\033[1;35mDEBUG\033[0m ", 
    "\033[1;35mDEBUG1\033[0m ", 
    "\033[1;35mDEBUG2\033[0m ", 
    "\033[1;35mDEBUG3\033[0m ", 
    "\033[1;35mDEBUG4\033[0m "
    };
class logIt
{
public:
    logIt(loglevel_e _loglevel = logERROR,  const char* func = "", int line= 0) {
        _buffer << loglevel_str[_loglevel] << ":"  << func << ":" << line << ":"
            << std::string(
                _loglevel > logDEBUG 
                ? (_loglevel - logDEBUG) * 4 
                : 1
                , ' ');
    }

    template <typename T>
    logIt & operator<<(T const & value)
    {
        _buffer << value;
        return *this;
    }

    ~logIt()
    {
        _buffer << std::endl;
        // This is atomic according to the POSIX standard
        // http://www.gnu.org/s/libc/manual/html_node/Streams-and-Threads.html
        std::cerr << _buffer.str();
    }

private:
    std::ostringstream _buffer;
};

extern loglevel_e loglevel;

#define log(level) \
if (level > loglevel) ; \
else logIt(level, __func__, __LINE__)


void log_test(){
    loglevel_e loglevel_save = loglevel;

    loglevel = logINFO;

    log(logINFO) << "foo " << "bar " << "baz";

    log(logERROR) << "logERROR";
    log(logWARNING) << "logWARNING";
    log(logINFO) << "logINFO";
    log(logDEBUG) << "logDEBUG";
    log(logDEBUG1) << "logDEBUG1";
    log(logDEBUG2) << "logDEBUG2";
    log(logDEBUG3) << "logDEBUG3";

    loglevel = loglevel_save;
}