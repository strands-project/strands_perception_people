/*
 * tictoc.hpp
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#ifndef __fovis_tictoch_hpp__
#define __fovis_tictoch_hpp__

#include <inttypes.h>
#include <vector>
#include <string>

/**
 * @brief quick and dirty profiling tool.
 * @ingroup BotCoreTime
 *
 * inspired by the matlab tic/toc command
 *
 * call tictoc("description") to set the timer going
 * call it again with the same description to stop the timer
 *
 * Note: To get output, set the "FOVIS_TICTOC" environment variable to something
 *
 * @{
 */

namespace fovis
{

#define FOVIS_TICTOC_ENV "FOVIS_TICTOC"

/**
 * Structure to keep track of timing information for each tictoc entry.
 */
struct tictoc_t
{
  int64_t t;
  int64_t totalT;
  int64_t ema;
  int64_t min;
  int64_t max;
  int numCalls;
  char flag;
  std::string description;
};

/**
 * tictoc:
 *
 * basic invocation, the second time its called, it returns the time difference in microseconds
 **/
int64_t
tictoc(const char *description);

/**
 * tictoc_full:
 *
 * full invocation, allows you to specify an
 * exponential moving average rate, and the current EMA value is returned in the ema argument
 */
int64_t
tictoc_full(const char *description, double ema_alpha, int64_t * ema);

/**
 * tictoc_sort_type_t:
 *
 * Different Options for sorting the printed results
 */
typedef enum
{
    TICTOC_AVG,
    TICTOC_TOTAL,
    TICTOC_MIN,
    TICTOC_MAX,
    TICTOC_EMA,
    TICTOC_ALPHABETICAL
} tictoc_sort_type_t;

/**
 * tictoc_print_stats:
 *
 * Print Out the stats from tictoc
 */
void
tictoc_print_stats(tictoc_sort_type_t sortType);

/**
 * Get tictoc entries.
 */
void
tictoc_get_stats(std::vector<tictoc_t> *stats);

/**
 * @}
 */

}

#endif
