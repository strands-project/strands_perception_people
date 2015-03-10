#ifndef ASSO_EXCEPTION_H
#define ASSO_EXCEPTION_H

#include <exception>
using namespace std;

class asso_exception: public exception
{
  virtual const char* what() const throw()
  {
    return "Unknown association algorithm!";
  }
};

class filter_exception: public exception
{
  virtual const char* what() const throw()
  {
    return "Unknown filter type!";
  }
};

#endif // ASSO_EXCEPTION_H
