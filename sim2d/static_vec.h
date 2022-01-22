#pragma once
#include <iostream>

// from rp source
namespace rp {
  template<typename Scalar_, int Dim_>
  class Vec_ {
  public:
    using Scalar=Scalar_;
    static constexpr int Dim=Dim_;
    using ThisType=Vec_<Scalar, Dim>;

    ThisType& operator+=(const ThisType& other) {
      for (int i=0; i<Dim; ++i)
        _values[i]+=other._values[i];
      return *this;
    }
    
    ThisType& operator-=(const ThisType& other) {
      for (int i=0; i<Dim; ++i)
        _values[i]-=other._values[i];
      return *this;
    }
    
    ThisType& operator*=(const float& scalar) {
      for (int i=0; i<Dim; ++i)
        _values[i]*=scalar;
      return *this;
    }

    ThisType operator*(const float& scalar) {
      ThisType v(*this);
      return v*=scalar;
    }

    
    Scalar dot(const ThisType& other) const {
      Scalar acc=Scalar(0);
      for (int i=0; i<Dim; ++i)
        acc +=_values[i]*other._values[i];
      return acc;
    }

    Scalar squaredNorm() const {
      return dot(*this);
    }
    Scalar norm(const ThisType& other) const {
      return sqrt(squaredNorm());
    }
    
    ThisType operator+(const ThisType& other) const {
      ThisType r(*this);
      r+=other;
      return r;
    }

    ThisType operator-(const ThisType& other) const {
      ThisType r(*this);
      r-=other;
      return r;
    }
    const int dimension() const {return Dim;}
    Scalar& at(int pos) {return _values[pos];}
    const Scalar& at(int pos) const {return _values[pos];}

    inline Scalar& operator[](int pos) {
      return _values[pos];
    }
    inline const Scalar& operator[](int pos) const {
      return _values[pos];
    }
    // removed protected so I can init w curly braces 
    //https://en.cppreference.com/w/cpp/language/aggregate_initialization
    Scalar _values[Dim_];
  };
  

  template<typename Scalar_, int Dim_>
  std::ostream&  operator << (std::ostream& os, const Vec_<Scalar_, Dim_>& v) {
    os << "[ ";
    for (int i=0; i<v.dimension(); ++i) {
      os << v.at(i) << " ";
    }
    os << "]";
    return os;
  }
  
}