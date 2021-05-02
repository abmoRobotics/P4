#include <array>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

namespace vector{

    template <class T>
    T dotProd(std::array<T,3>,std::array<T,3>);

    // template <class T>
    // std::array<T,3> crossProd(std::array<T,3>,std::array<T,3>);

    template <class T>
    std::array<T,3> sub(std::array<T,3>,std::array<T,3>);

    template <class T>
    std::array<T,3> add(std::array<T,3>,std::array<T,3>);

    template <class T>
    std::array<T,3> scalarProd(T,std::array<T,3>);

    template <class T>
    std::array<T,3> vecProj(std::array<T,3>,std::array<T,3>);

    template <class T>
    std::array<T,3> pointToArray(geometry_msgs::Vector3);

    template <class T>
    std::array<T,3> pointToArray(geometry_msgs::Point);

    template <class T>
    T length(std::array<T,3>);
}