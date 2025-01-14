// variable.hpp
#ifndef VARIABLE_HPP
#define VARIABLE_HPP

#include <cstdint> // Pour std::int32_t

enum class ParkingType : std::int32_t {
    NONE = 0,
    PARALLEL = 1,  
    PERPENDICULAR = 2, 
    ANGLED = 3 
};

#endif // VARIABLE_HPP