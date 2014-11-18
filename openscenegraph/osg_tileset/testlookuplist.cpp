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

#include <cassert>
#include <iostream>

#include <LookupList.h>

scratch::LookupList<std::string,std::string,std::map> lkls;

void reset(std::string s)
{
    lkls.clear();
    for(size_t i=0; i < s.size(); i++) {
        lkls.insert(lkls.end(),std::make_pair(s.substr(i,1),s));
    }
}

void test_insert()
{
    std::cout << "test_insert... " << std::endl;

    lkls.clear();

    // duplicates
    std::string const test = "#";
    auto it1 = lkls.insert(lkls.end(),std::make_pair(test,test));
    auto it2 = lkls.insert(lkls.end(),std::make_pair(test,test));
    assert(it1 == it2);

    // copy
    std::pair<std::string,std::string> ins;
    ins.second = test;
    ins.first = "A";
    lkls.insert(lkls.end(),ins);
    ins.first = "B";
    lkls.insert(lkls.end(),ins);
    ins.first = "C";
    lkls.insert(lkls.end(),ins);
    ins.first = "D";
    lkls.insert(lkls.end(),ins);

    // move
    lkls.insert(lkls.begin(),std::make_pair(std::string("W"),std::string("0")));
    lkls.insert(lkls.begin(),std::make_pair(std::string("X"),std::string("0")));
    lkls.insert(lkls.begin(),std::make_pair(std::string("Y"),std::string("0")));
    lkls.insert(lkls.begin(),std::make_pair(std::string("Z"),std::string("0")));

    std::string const expect = "ZYXW#ABCD";
    std::string result;
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }

    assert(result == expect);
}

void test_erase()
{
    std::cout << "test_erase..." << std::endl;

    lkls.clear();

    // "SHAPE"
    lkls.insert(lkls.end(),std::make_pair(std::string("S"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("H"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("A"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("P"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("E"),std::string("0")));

    std::string expect = "SHAPE";
    std::string result;
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // erase with iterator: expect "HAP"
    lkls.erase(lkls.begin());
    lkls.erase(lkls.last());

    expect = "HAP";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // erase with key: expect "A"
    lkls.erase(std::string("P"));
    lkls.erase(std::string("H"));

    expect = "A";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);
}

void test_move()
{
    std::cout << "test_move..." << std::endl;
    lkls.clear();

    // SATURN
    lkls.insert(lkls.end(),std::make_pair(std::string("S"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("A"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("T"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("U"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("R"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("N"),std::string("0")));

    std::string expect = "SATURN";
    std::string result;
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // rearrange "SATURN" into "ARTSUN"
    lkls.move(lkls.find("A"),lkls.begin());
    lkls.move(lkls.find("R"),lkls.find("T"));
    lkls.move(lkls.find("S"),lkls.find("U"));

    expect = "ARTSUN";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);
}

void test_find()
{
    std::cout << "test_find..." << std::endl;
    lkls.clear();

    lkls.insert(lkls.end(),std::make_pair(std::string("H"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("A"),std::string("0")));
    lkls.insert(lkls.end(),std::make_pair(std::string("T"),std::string("0")));

    assert(lkls.find("H")==lkls.begin());
    assert(lkls.find("T")==lkls.last());
    assert(lkls.find("Z")==lkls.end());
}

void test_trim()
{
    std::cout << "test_trim..." << std::endl;
    lkls.clear();

    std::string expect;
    std::string result;

    // trim with size > lkls.size: expect no change
    reset("ASTRO");
    lkls.trim(lkls.size()+5);

    expect = "ASTRO";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // trim with size = 3: expect "AST"
    reset("ASTRO");
    lkls.trim(3);

    expect = "AST";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);


    // trim with size == position: expect "AST"
    reset("ASTRO");
    lkls.trim(lkls.find("T"),3);

    expect = "AST";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // trim with size < position: expect "AS"
    reset("ASTRO");
    lkls.trim(lkls.find("T"),2);

    expect = "AS";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);

    // trim with size > position: expect "ASTR"
    reset("ASTRO");
    lkls.trim(lkls.find("S"),4);

    expect = "ASTR";
    result.clear();
    for(auto it = lkls.begin();
        it != lkls.end(); ++it)
    {
        result.append(it->first);
    }
    assert(result == expect);
}

int main()
{
    test_insert();
    test_erase();
    test_move();
    test_find();
    test_trim();

    std::cout << "[ALL OK]" << std::endl;

    return 0;
}
