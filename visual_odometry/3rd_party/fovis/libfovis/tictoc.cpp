/*
 * tictoc.c
 *
 *  Created on: May 29, 2009
 *      Author: abachrac
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include "tictoc.hpp"

//simple, quick and dirty profiling tool...

namespace fovis
{

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static bool
_tictoc_t_avgTimeCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return (t1->totalT / t1->numCalls) < (t2->totalT / t2->numCalls);
}
static bool
_tictoc_t_totalTimeCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->totalT < t2->totalT;
}
static bool
_tictoc_t_maxTimeCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->max < t2->max;
}
static bool
_tictoc_t_minTimeCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->min > t2->min;
}
static bool
_tictoc_t_emaTimeCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return t1->ema < t2->ema;
}

static bool
_tictoc_t_alphCompare(const tictoc_t* t1, const tictoc_t* t2)
{
    if (t1->numCalls < 1)
        return 1;
    else if (t2->numCalls < 1)
        return 0;
    else
        return strcmp(t1->description.c_str(), t2->description.c_str());
}

static int _tictoc_enabled = 1;
static int _tictoc_initialized = 0;
typedef std::map<std::string, tictoc_t> TictocMap;
static TictocMap _tictoc_map;

static void
_initializeTictoc()
{
    const char *tmp;
    tmp = getenv(FOVIS_TICTOC_ENV);
    if (tmp != NULL) {
        _tictoc_enabled = 1;
    } else {
        _tictoc_enabled = 0;
    }
}

int64_t
tictoc(const char *description)
{
    return tictoc_full(description, .01, NULL);
}

int64_t
tictoc_full(const char *description, double ema_alpha, int64_t * ema)
{
    if (!_tictoc_enabled)
        return 0;

    int64_t ret = 0;

    if (!_tictoc_initialized) {
        _tictoc_initialized = 1;
        _initializeTictoc();
        if (!_tictoc_enabled) {
            return 0;
        }

    }

    int64_t tictoctime = _timestamp_now();
    TictocMap::iterator eiter = _tictoc_map.find(description);
    if(eiter == _tictoc_map.end()) {
        //first time around, allocate and set the timer goin...
        tictoc_t entry;
        entry.flag = 1;
        entry.t = tictoctime;
        entry.totalT = 0;
        entry.numCalls = 0;
        entry.max = -1e15;
        entry.min = 1e15;
        entry.ema = 0;
        entry.description = description;
        _tictoc_map[description] = entry;
        ret = tictoctime;
    } else if (eiter->second.flag == 0) {
        eiter->second.flag = 1;
        eiter->second.t = tictoctime;
        ret = tictoctime;
    } else {
        eiter->second.flag = 0;
        int64_t dt = tictoctime - eiter->second.t;
        eiter->second.numCalls++;
        eiter->second.totalT += dt;
        if (dt < eiter->second.min)
            eiter->second.min = dt;
        if (dt > eiter->second.max)
            eiter->second.max = dt;
        eiter->second.ema = (1.0 - ema_alpha) * eiter->second.ema + ema_alpha * dt;
        if (ema != NULL)
            *ema = eiter->second.ema;
        ret = dt;
    }
    return ret;
}

void
tictoc_get_stats(std::vector<tictoc_t> *stats)
{
    if (!_tictoc_enabled) {
      return;
    }
    TictocMap::iterator iter = _tictoc_map.begin();
    TictocMap::iterator eiter = _tictoc_map.end();
    for(; iter != eiter; ++iter) {
      stats->push_back(iter->second);
    }
}

void
tictoc_print_stats(tictoc_sort_type_t sortType)
{
  if (!_tictoc_enabled) {
    return;
  }
  TictocMap::iterator iter = _tictoc_map.begin();
  TictocMap::iterator eiter = _tictoc_map.end();
  std::vector<const tictoc_t*> entries;
  for(; iter != eiter; ++iter) {
    entries.push_back(&iter->second);
  }

    printf("\n--------------------------------------------\n");
    printf("tictoc Statistics, sorted by ");
    switch (sortType)
    {
      case TICTOC_AVG:
        printf("average time\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_avgTimeCompare);
        break;
      case TICTOC_MIN:
        printf("min time\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_minTimeCompare);
        break;
      case TICTOC_MAX:
        printf("max time\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_maxTimeCompare);
        break;
      case TICTOC_EMA:
        printf("EMA time\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_emaTimeCompare);
        break;
      case TICTOC_TOTAL:
        printf("total time\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_totalTimeCompare);
        break;
      case TICTOC_ALPHABETICAL:
        printf("alphabetically\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_alphCompare);
        break;
      default:
        fprintf(stderr, "WARNING: invalid sort type in tictoc, using AVG\n");
        std::sort(entries.begin(), entries.end(), _tictoc_t_avgTimeCompare);
        break;
    }
    printf("--------------------------------------------\n");
    for(std::vector<const tictoc_t*>::iterator iter=entries.begin();
        iter != entries.end(); ++iter) {
      const tictoc_t* tt = *iter;
      if (tt->numCalls < 1)
        return;
      double totalT = (double) tt->totalT / 1.0e6;
      double avgT = ((double) tt->totalT / (double) tt->numCalls) / 1.0e6;
      double minT = (double) tt->min / 1.0e6;
      double maxT = (double) tt->max / 1.0e6;
      double emaT = (double) tt->ema / 1.0e6;
      printf("%30s: \t numCalls = %d \t totalT=%.2f \t avgT=%.4f \t minT=%.4f \t maxT=%.4f \t emaT=%.4f\n",
          tt->description.c_str(), tt->numCalls, totalT, avgT, minT, maxT, emaT);
    }
    printf("--------------------------------------------\n");
}

}
