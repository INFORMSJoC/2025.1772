#ifndef _FILO2_STRINGUTILS_HPP_
#define _FILO2_STRINGUTILS_HPP_

#include <algorithm>
#include <string>

inline std::string get_basename(const std::string& path) {
    return {std::find_if(path.rbegin(), path.rend(),
                         [](char c) {
                             return c == '/';
                         })
                .base(),
            path.end()};
}

#endif