#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "internal_utils.hpp"

namespace fovis
{

bool
optionsGetInt(const VisualOdometryOptions& options, std::string name,
    int* result)
{
  VisualOdometryOptions::const_iterator iter = options.find(name);
  if(iter == options.end()) {
    fprintf(stderr, "Option [%s] not specified!\n", name.c_str());
    return false;
  }
  std::string val = iter->second;
  char* eptr = NULL;
  int v = strtol(val.c_str(), &eptr, 0);
  if(*eptr != '\0') {
    fprintf(stderr, "Illegal value of [%s] for integer option [%s]\n",
        val.c_str(), name.c_str());
    return false;
  }
  *result = v;
  return true;
}

bool
optionsGetBool(const VisualOdometryOptions& options, std::string name,
    bool* result)
{
  VisualOdometryOptions::const_iterator iter = options.find(name);
  if(iter == options.end()) {
    fprintf(stderr, "Option [%s] not specified!\n", name.c_str());
    return false;
  }
  std::string val = iter->second;
  std::string val_copy(val);
  // trim string and ignore case.
  val_copy.erase(std::remove_if(val_copy.begin(), val_copy.end(), ::isspace), val_copy.end());
  // ignore case.
  std::transform(val_copy.begin(), val_copy.end(), val_copy.begin(), ::tolower);
  if (val_copy == "true") {
    *result = true;
  } else if (val_copy == "false") {
    *result = false;
  } else {
    fprintf(stderr, "Illegal value of [%s] for boolean option [%s]\n",
        val.c_str(), name.c_str());
    return false;
  }
  return true;
}

int
optionsGetIntOrFromDefault(const VisualOdometryOptions& options, std::string name,
    const VisualOdometryOptions& defaults)
{
  int result;
  if(optionsGetInt(options, name, &result))
    return result;
  bool got_opt = optionsGetInt(defaults, name, &result);
  if(!got_opt) {
    assert(false);
  }
  fprintf(stderr, "Using default value of [%d] for option [%s]\n", result, name.c_str());
  return result;
}

bool
optionsGetBoolOrFromDefault(const VisualOdometryOptions& options, std::string name,
    const VisualOdometryOptions& defaults)
{
  bool result;
  if(optionsGetBool(options, name, &result))
    return result;
  bool got_opt = optionsGetBool(defaults, name, &result);
  if(!got_opt) {
    assert(false);
  }
  fprintf(stderr, "Using default value of [%s] for option [%s]\n", (result? "true" : "false"), name.c_str());
  return result;
}

bool
optionsGetDouble(const VisualOdometryOptions& options, std::string name,
    double* result)
{
  VisualOdometryOptions::const_iterator iter = options.find(name);
  if(iter == options.end()) {
    fprintf(stderr, "Option [%s] not specified!\n", name.c_str());
    return false;
  }
  std::string val = iter->second;
  char* eptr = NULL;
  double v = strtod(val.c_str(), &eptr);
  if(*eptr != '\0') {
    fprintf(stderr, "Illegal value of [%s] for float option [%s]\n",
        val.c_str(), name.c_str());
    return false;
  }
  *result = v;
  return true;
}

double
optionsGetDoubleOrFromDefault(const VisualOdometryOptions& options, std::string name,
    const VisualOdometryOptions& defaults)
{
  double result;
  if(optionsGetDouble(options, name, &result))
    return result;
  bool got_opt = optionsGetDouble(defaults, name, &result);
  if(!got_opt) {
    assert(false);
  }
  fprintf(stderr, "Using default value of [%f] for option [%s]\n", result, name.c_str());
  return result;
}

}
