#include <iostream>
#include <vector>
#include "customClass.hpp"

//#define DEBUG

CustomClass::CustomClass(int a, int b, int c) {
    #ifdef DEBUG
        std::cout << "default constructor" << std::endl;
    #endif

        m_data.push_back(a);
        m_data.push_back(b);
        m_data.push_back(c);
}

CustomClass::CustomClass(const CustomClass &other) {
    #ifdef DEBUG
        std::cout << "copy constructor" << std::endl;
    #endif

        m_data.push_back(other.m_data[0]);
        m_data.push_back(other.m_data[1]);
        m_data.push_back(other.m_data[2]);
}

// CustomClass::CustomClass(CustomClass &&other) {
//     #ifdef DEBUG
//         std::cout << "move cosntructor" << std::endl;
//     #endif

//         m_data[0] = other.m_data[0];
//         m_data[1] = other.m_data[1];
//         m_data[2] = other.m_data[2];
//         other.m_data.clear();
// }

CustomClass &CustomClass::operator=(const CustomClass &other) {
    #ifdef DEBUG
        std::cout << "copy assignment operator" << std::endl;
    #endif

    if (this != &other)
    {
        m_data[0] = other.m_data[0];
        m_data[1] = other.m_data[1];
        m_data[2] = other.m_data[2];
    }
    return *this;
}

// CustomClass &CustomClass::operator=(CustomClass &&other) {
//     #ifdef DEBUG
//         std::cout << "move assignment operator" << std::endl;
//     #endif

//     if (this != &other)
//     {
//         m_data[0] = other.m_data[0];
//         m_data[1] = other.m_data[1];
//         m_data[2] = other.m_data[2];
//         other.m_data.clear();
//     }
//     return *this;
// }

CustomClass::~CustomClass() {
    #ifdef DEBUG
        std::cout << "destructor" << std::endl;
    #endif
}



bool operator==(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG
        std::cout << "operator ==" << std::endl;
    #endif
    return obj1.m_data[0] == obj2.m_data[0] && obj1.m_data[1] == obj2.m_data[1] && obj1.m_data[2] == obj2.m_data[2];
}


bool operator!=(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG
        std::cout << "operator !=" << std::endl;
    #endif
    return !(obj1 == obj2);
}

bool operator<(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG
        std::cout << "operator <" << std::endl;
    #endif
    return obj1.m_data[0] < obj2.m_data[0] && obj1.m_data[1] < obj2.m_data[1] && obj1.m_data[2] < obj2.m_data[2];
}

std::ostream& operator<<(std::ostream& out, const CustomClass& obj)
{
    out << "["<<obj.m_data[0]<<" "<<obj.m_data[1]<<" "<<obj.m_data[2]<<"]";
    return out;
}


std::istream& operator>>(std::istream& is, CustomClass& obj)
{
    char c1,c2;
    int a,b,c;
    is >> c1 >> a >> b >> c >> c2;
    if( c1 != '[' || c2 != ']' )
        is.setstate(std::ios::failbit);
    obj.m_data[0]  = a;
    obj.m_data[1]  = b;
    obj.m_data[2]  = c;
    
    return is;
}