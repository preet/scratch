#include <iostream>

#define GLM_SWIZZLE_NONE
#define GLM_FORCE_CXX98
#include <glm/glm.hpp>

#pragma message "Hello WOrld"


int main(int argc, char *argv[])
{
//    std::cout << "GLM Compiler is: "
//              << GLM_COMPILER << std::endl;
    return 0;
}



// constructing unordered_maps
//#include <iostream>
//#include <string>
//#include <unordered_map>

//#include <QDebug>

//typedef std::unordered_map<std::string,std::string> stringmap;

//stringmap merge (stringmap a,stringmap b) {
//  stringmap temp(a); temp.insert(b.begin(),b.end()); return temp;
//}

//int main ()
//{
//  stringmap first;                              // empty
//  stringmap second ( {{"apple","red"},{"lemon","yellow"}} );       // init list
//  stringmap third ( {{"orange","orange"},{"strawberry","red"}} );  // init list
//  stringmap fourth (second);                    // copy
//  stringmap fifth (merge(third,fourth));        // move
//  stringmap sixth (fifth.begin(),fifth.end());  // range

//  std::cout << "sixth contains:";
  
//  std::unordered_map<std::string,std::string>::iterator sIt;
//  for(sIt = sixth.begin(); sIt != sixth.end(); ++sIt)
//  {
//     std::cout << " " << sIt->first << ":" << sIt->second;
//     std::cout << std::endl;
//  }

//  return 0;
//}


//#include <QApplication>
//#include <QPushButton>
//#include <QString>

//#include <iostream>
//#include <boost/unordered/unordered_map.hpp>


//int main(int argc, char *argv[])
//{

//    std::cout << "Hello World" << std::endl;

//    boost::unordered_map<std::string,unsigned int> mapNumberNames;
//    std::pair<std::string,unsigned int> numberName;

//    numberName.first = "zero";
//    numberName.second = 0;
//    mapNumberNames.insert(numberName);

//    numberName.first = "one";
//    numberName.second = 1;
//    mapNumberNames.insert(numberName);

//    numberName.first = "two";
//    numberName.second = 2;
//    mapNumberNames.insert(numberName);

//    numberName.first = "three";
//    numberName.second = 3;
//    mapNumberNames.insert(numberName);

//    boost::unordered_map<std::string,unsigned int>::iterator mIt;
//    mIt = mapNumberNames.find("two");

//    if(mIt == mapNumberNames.end())   {
//        std::cout << "Something went wrong!" << std::endl;
//        buttonMsg = "Something went wrong!";
//    }
//    else   {
//        std::cout << "The word " << mIt->first
//                  << " represents the number " << mIt->second << std::endl;
//        buttonMsg = "We found the number 2!";
//    }

//    QString buttonMsg;
//    QApplication myApp(argc,argv);
//    QPushButton myButton(buttonMsg);
//    QObject::connect(&myButton,SIGNAL(clicked()),
//                     &myApp,SLOT(quit()));
//    myButton.show();
//    return myApp.exec();
//}
