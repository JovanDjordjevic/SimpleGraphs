#ifndef __CUSTOM_CLASS__
#define __CUSTOM_CLASS__

#include <iostream>
#include <vector>


class CustomClass{
    public:
        CustomClass(int a=0, int b=0, int c=0);
        CustomClass(const CustomClass& other);
        //CustomClass(CustomClass&& other);

        CustomClass& operator=(const CustomClass& other);
        //CustomClass& operator=(CustomClass&& other);

        ~CustomClass();

        // i don't know why yet, but these must be friend (even though class has no private data) or compilation fails
        friend bool operator==(const CustomClass& obj1, const CustomClass& obj2);
        friend std::ostream& operator<<(std::ostream& out, const CustomClass& obj);
        friend std::istream& operator>>(std::istream& is, CustomClass& obj);
        
    public:
        std::vector<int> m_data;
};


// i don't know why yet, but std::hash overload must either be here or in main.cpp or compilation fails
namespace std{
    template<>
    struct hash<CustomClass>{
        size_t operator()(const CustomClass& obj) const{
            std::hash<int> hasher;
            return hasher(obj.m_data[0]) + hasher(obj.m_data[1]) + hasher(obj.m_data[2]);
        }
    };
}


#endif //__CUSTOM_CLASS__