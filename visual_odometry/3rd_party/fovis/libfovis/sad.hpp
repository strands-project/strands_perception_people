#ifndef __fovis_sad_hpp__
#define __fovis_sad_hpp__

#ifndef USE_SSE
#define USE_SSE
#endif

#ifdef USE_SSE
#include <emmintrin.h>
#endif

namespace fovis
{

/**
 *
 * \brief Calculates the Sum of Absolute Deviations (SAD) score between two vectors
 * of length descriptor_len.
 *
 */
class SAD {
public:
  SAD(int descriptor_len) :
      _descriptor_len(descriptor_len),
      _nsad_ops(descriptor_len / 16 + ((descriptor_len % 16) ? 1 : 0))
      { }

  /**
   * Calculate SAD score between ref_desc and target_desc. ref_desc
   * and target_desc must be padded with zero-filled pad bytes up to
   * multiple of 16.
   */
  int32_t score(const uint8_t *ref_desc, const uint8_t *target_desc) {
#ifdef USE_SSE
    // compute sum of absolute differences (fast)
    const uint8_t * pp = ref_desc;
    const uint8_t * cp = target_desc;
    __m128i d = _mm_setzero_si128();
    for (int i = 0; i < _nsad_ops; i++) {
      __m128i c = _mm_sad_epu8(*(__m128i *) pp, *(__m128i *) cp);
      d = _mm_add_epi16(c, d);
      pp += 16;
      cp += 16;
    }

    __m128i e = _mm_srli_si128(d, 8);
    __m128i f = _mm_add_epi32(d, e);

    int32_t score = _mm_cvtsi128_si32(f);
#else
    int32_t score = 0;
    for(int i=0; i<descriptor_len; i++) {
      score += abs(ref_desc[i] - target_desc[i]);
    }
#endif
    return score;
  }

  int getWorstScore() const {
    return _descriptor_len * 255;
  }

private:
  int _descriptor_len;
  int _nsad_ops;
};

}

#endif
