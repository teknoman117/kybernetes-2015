#include "utility.hpp"

#include <algorithm>
#include <iterator>
#include <regex>
#include <vector>

// Tokenize a string
void tokenize(const std::string& input, const std::string& regexp, std::vector<std::string>& output)
{
    std::regex regex(regexp);
    auto begin = std::sregex_token_iterator(input.begin(), input.end(), regex, -1);
    auto end = std::sregex_token_iterator();
    output.resize(distance(begin, end));
    std::copy(begin, end, output.begin());
}
