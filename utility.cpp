#include "utility.hpp"

#include <algorithm>
#include <iterator>
#include <regex>
#include <vector>

void tokenize(const std::string& input, const std::string& regexp, std::vector<std::string>& output)
{
    // passing -1 as the submatch index parameter performs splitting
    std::regex regex(regexp);
    std::copy(std::sregex_token_iterator(input.begin(), input.end(), regex, -1),
              std::sregex_token_iterator(),
              output.end());
}
