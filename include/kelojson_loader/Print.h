#ifndef KELO_KELOJSON_PRINT_H
#define KELO_KELOJSON_PRINT_H

#include <string>

namespace kelo {
namespace kelojson {

struct Print
{
    static constexpr const char* Err     = "\033[31m";    // start red
    static constexpr const char* Warn    = "\033[33m";    // start yellow
    static constexpr const char* Success = "\033[32m";    // start green
    static constexpr const char* End     = "\033[0m";     // back to normal

};

} // namespace kelojson
} // namespace kelo
#endif // KELO_KELOJSON_PRINT_H
