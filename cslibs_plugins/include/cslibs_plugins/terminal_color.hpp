#ifndef CSLIBS_PLUGINS_TERMINAL_COLOR
#define CSLIBS_PLUGINS_TERMINAL_COLOR

#include <string>

namespace cslibs_plugins {
namespace io {
namespace color {
//the following are UBUNTU/LINUX, and MacOS ONLY terminal color codes.
#define RESET       std::string{"\033[0m" }
#define BLACK       std::string{"\033[30m"}      /* Black */
#define RED         std::string{"\033[31m"}      /* Red */
#define GREEN       std::string{"\033[32m"}      /* Green */
#define YELLOW      std::string{"\033[33m"}      /* Yellow */
#define BLUE        std::string{"\033[34m"}      /* Blue */
#define MAGENTA     std::string{"\033[35m"}      /* Magenta */
#define CYAN        std::string{"\033[36m"}      /* Cyan */
#define WHITE       std::string{"\033[37m"}      /* White */
#define BOLD        std::string{"\033[1m" }      /* Bold */

template<typename T> inline std::string color(const T & out, const std::string &color)
    { return color + std::to_string(out) + RESET;}
template<> inline std::string color<std::string>(const std::string & out, const std::string &color) 
    { return color + out + RESET;}
template<typename T> inline std::string black(const T &out)  { return color(out, BLACK); }
template<typename T> inline std::string red(const T &out)    { return color(out, RED); }
template<typename T> inline std::string green(const T &out)  { return color(out, GREEN); }
template<typename T> inline std::string yellow(const T &out) { return color(out, YELLOW); }
template<typename T> inline std::string blue(const T &out)   { return color(out, BLUE); }
template<typename T> inline std::string magenta(const T &out){ return color(out, MAGENTA); }
template<typename T> inline std::string cyan(const T &out)   { return color(out, CYAN); }
template<typename T> inline std::string white(const T &out)  { return color(out, WHITE); }
inline std::string bold(const std::string &out) { return BOLD + out; }

}
}
}

#endif // CSLIBS_PLUGINS_TERMINAL_COLOR