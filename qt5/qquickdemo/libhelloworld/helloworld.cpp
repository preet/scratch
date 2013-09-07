#include "helloworld.h"


HelloWorld::HelloWorld() :
    m_list_idx(0)
{
    m_list_quotes.push_back("Evil when we are in its power is not felt as evil but as a necessity, or even a duty.");
    m_list_quotes.push_back("Without the aid of prejudice and custom I should not be able to find my way across the room.");
    m_list_quotes.push_back("One of the keys to happiness is a bad memory.");
    m_list_quotes.push_back("Anyone who goes to a psychiatrist ought to have his head examined.");
    m_list_quotes.push_back("The most efficient labor-saving device is still money.");
    m_list_quotes.push_back("One doesn't discover new lands without consenting to lose sight of the shore for a very long time.");
    m_list_quotes.push_back("Some are born great, some achieve greatness, and some hire public relations officers.");
}

std::string HelloWorld::getQuote()
{
    m_list_idx++;

    if(m_list_idx > m_list_quotes.size()-1)   {
        m_list_idx=0;
    }

    return m_list_quotes[m_list_idx];
}
