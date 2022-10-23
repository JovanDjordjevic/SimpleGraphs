#ifndef __CUSTOM_CLASS__
#define __CUSTOM_CLASS__

#include <iostream>
#include <vector>

// In order for a custom class to be used a node in SimpleGraphs, it must have the following implemented:
//      - default constructor
//      - copy constructor
//      - move constructor          (works without this, but this may improve performance)
//      - copy assignment operator
//      - move asignment operator   (works without this, but this may improve performance)
//      - operator==
//      - operator<
//      - std::hash specialization for the custom class
//      - operator<<                (only needed if printing of a node to an output stream/file is needed by user)
//      - operator>>                (only needed if it will be loaded from an input stream/file)


class CustomClass{
    public:
        CustomClass(int a = 0, int b = 0, int c = 0);
        CustomClass(const CustomClass& other);
        CustomClass(CustomClass&& other);

        CustomClass& operator=(const CustomClass& other);
        CustomClass& operator=(CustomClass&& other);

        ~CustomClass();

        // i don't know why yet, but these must be friend (even though class has no private data) or compilation fails
        friend bool operator==(const CustomClass& obj1, const CustomClass& obj2);
        friend bool operator!=(const CustomClass& obj1, const CustomClass& obj2);
        friend bool operator<(const CustomClass& obj1, const CustomClass& obj2);
        friend std::ostream& operator<<(std::ostream& out, const CustomClass& obj);
        friend std::istream& operator>>(std::istream& is, CustomClass& obj);
        
    public:
        std::vector<int> m_data;
};

namespace std{
    template<>
    struct hash<CustomClass>{
        size_t operator()(const CustomClass& obj) const{
            std::hash<int> hasher;
            return hasher(obj.m_data[0]) + hasher(obj.m_data[1]) + hasher(obj.m_data[2]);
        }
    };
} // namespace std

#endif //__CUSTOM_CLASS__