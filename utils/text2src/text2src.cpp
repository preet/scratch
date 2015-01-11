/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

// sys
#include <cstdint>

// stl
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    // Expects a single argument with the path of
    // the text file to be read in
    if(argc != 2) {
        std::cout << "ERROR: Incorrect number of args" << std::endl;
        return -1;
    }

    // Read in file
    std::string const file_path(argv[1]);
    std::ifstream file(file_path);
    if(!(file.is_open() && file.good())) {
        std::cout << "ERROR: Failed to open file: " << file_path << std::endl;
    }

    std::string line;
    while(std::getline(file,line)) {
        for(auto it = line.begin(); it != line.end();)
        {
            if(*it == '"') {
                it = line.insert(it,'\\');
                ++it;
            }
            ++it;
        }

        std::cout << "\"" << line << "\\n\"" <<std::endl;
    }

    return 0;
}
