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

#endif // ASSO_EXCEPTION_H
