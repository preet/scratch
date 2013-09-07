#ifndef HELLOWORLD_H
#define HELLOWORLD_H

#include <vector>
#include <string>

class HelloWorld
{
    
public:
    HelloWorld();

    std::string getQuote();

private:
    size_t m_list_idx;
    std::vector<std::string> m_list_quotes;
};

#endif // HELLOWORLD_H
