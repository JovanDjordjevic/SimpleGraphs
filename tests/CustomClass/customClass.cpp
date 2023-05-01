#include "customClass.hpp"
#include <iostream>
#include <vector>

// #define DEBUG_DEFAULT_CONSTRUCTOR
// #define DEBUG_COPY_CONSTRUCTOR
// #define DEBUG_MOVE_CONSTRUCTOR
// #define DEBUG_COPY_ASSIGNMENT_OPERATOR
// #define DEBUG_MOVE_ASSIGNMENT_OPERATOR
// #define DEBUG_DESTRUCTOR
// #define DEBUG_OPERATOR_EQ
// #define DEBUG_OPERATOR_INEQ
// #define DEBUG_OPERATOR_LE

CustomClass::CustomClass(int a, int b, int c) {
    #ifdef DEBUG_DEFAULT_CONSTRUCTOR
        std::cout << "default constructor" << std::endl;
    #endif

        m_data.push_back(a);
        m_data.push_back(b);
        m_data.push_back(c);
}

CustomClass::CustomClass(const CustomClass &other) {
    #ifdef DEBUG_COPY_CONSTRUCTOR
        std::cout << "copy constructor" << std::endl;
    #endif

        m_data.push_back(other.m_data[0]);
        m_data.push_back(other.m_data[1]);
        m_data.push_back(other.m_data[2]);
}

CustomClass::CustomClass(CustomClass &&other) {
    #ifdef DEBUG_MOVE_CONSTRUCTOR
        std::cout << "move cosntructor" << std::endl;
    #endif

        m_data = std::move(other.m_data);
}

CustomClass &CustomClass::operator=(const CustomClass &other) {
    #ifdef DEBUG_COPY_ASSIGNMENT_OPERATOR
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

CustomClass &CustomClass::operator=(CustomClass &&other) {
    #ifdef DEBUG_MOVE_ASSIGNMENT_OPERATOR
        std::cout << "move assignment operator" << std::endl;
    #endif

    if (this != &other)
    {   
        m_data = std::move(other.m_data);
    }
    return *this;
}

CustomClass::~CustomClass() {
    #ifdef DEBUG_DESTRUCTOR
        std::cout << "destructor" << std::endl;
    #endif
}

bool operator==(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG_OPERATOR_EQ
        std::cout << "operator ==" << std::endl;
    #endif

    return obj1.m_data[0] == obj2.m_data[0] && obj1.m_data[1] == obj2.m_data[1] && obj1.m_data[2] == obj2.m_data[2];
}


bool operator!=(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG_OPERATOR_INEQ
        std::cout << "operator !=" << std::endl;
    #endif
    
    return !(obj1 == obj2);
}

bool operator<(const CustomClass& obj1, const CustomClass& obj2){
    #ifdef DEBUG_OPERATOR_LE
        std::cout << "operator <" << std::endl;
    #endif
    return obj1.m_data[0] < obj2.m_data[0] && obj1.m_data[1] < obj2.m_data[1] && obj1.m_data[2] < obj2.m_data[2];
}

std::ostream& operator<<(std::ostream& out, const CustomClass& obj)
{
    out << '[' << obj.m_data[0] << " " << obj.m_data[1] << " " << obj.m_data[2] << ']';
    return out;
}

std::istream& operator>>(std::istream& is, CustomClass& obj)
{
    char c1, c2;
    int a = 0, b = 0, c = 0;

    is >> c1 >> a >> b >> c >> c2;

    if( c1 != '[' || c2 != ']') {
        is.setstate(std::ios::failbit);
    }

    obj.m_data[0] = a;
    obj.m_data[1] = b;
    obj.m_data[2] = c;
    
    return is;
}