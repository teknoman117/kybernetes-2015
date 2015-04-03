#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <string>
#include <vector>

// Tokenize a string based on a regular expression
void tokenize(const std::string& input, const std::string& regexp, std::vector<std::string>& output);

#endif
