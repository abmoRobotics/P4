#include <libraries/customVector.h>

template <class T>
T vector::dotProd(std::array<T,3> A,std::array<T,3> B){
    T sum{0};
    for (size_t i = 0; i < A.size(); i++) sum = sum + (A[i]*B[i]);    
    return sum;
}

    template <class T>
std::array<T,3> vector::sub(std::array<T,3> A,std::array<T,3> B){
    std::array<T,3> vec;
    for (size_t i = 0; i < A.size(); i++) vec[i] = A[i] - B[i] ;    
    return vec;
}

template <class T>
std::array<T,3> vector::add(std::array<T,3> A,std::array<T,3> B){
    std::array<T,3> vec;
    for (size_t i = 0; i < A.size(); i++) vec[i] = A[i] + B[i] ;    
    return vec;
}

template <class T>
std::array<T,3> vector::scalarProd(T A,std::array<T,3> B){
    std::array<T,3> vec;
    for (size_t i = 0; i < B.size(); i++) vec[i] = A * B[i];
    return vec;
}

template <class T>
std::array<T,3> vector::vecProj(std::array<T,3> A,std::array<T,3> B){
    T lenB = vector::length(B);
    T lenBpow = std::pow(lenB,2);
    T dotAB = vector::dotProd(A,B);

    std::array<T,3> projVec = vector::scalarProd(dotAB,B);
    return projVec;
};

template <class T>
std::array<T,3> vector::pointToArray(geometry_msgs::Vector3 A){
    std::array<T,3> vec;
    vec[0] = A.x;
    vec[1] = A.y;
    vec[2] = A.z;
    return vec;
};

template <class T>
std::array<T,3> vector::pointToArray(geometry_msgs::Point A){
    std::array<T,3> vec;
    vec[0] = A.x;
    vec[1] = A.y;
    vec[2] = A.z;
    return vec;
};

template <class T>
std::array<T,3> vector::length(std::array<T,3> A){
    return std::sqrt(std::pow(A[0],2)+std::pow(A[1],2)+std::pow(A[2],2));
}
