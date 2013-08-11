#include "refine_feature_match.hpp"
#include "internal_utils.hpp"

#include "pyramid_level.hpp"

#ifndef USE_SSE
#define USE_SSE
#endif

#ifdef USE_SSE
#include <emmintrin.h>
#endif

namespace fovis
{

static inline int dot_int16_aligned(const int16_t* a, const int16_t* b, int num_taps)
{
#ifdef USE_SSE
  assert(FOVIS_IS_ALIGNED16(a) && FOVIS_IS_ALIGNED16(b));
  const int16_t* ap = a;
  const int16_t* bp = b;
  __m128i tmp;
  __m128i ma = _mm_load_si128((const __m128i *) a);
  __m128i mb = _mm_load_si128((const __m128i *) b);
  __m128i hi = _mm_mulhi_epi16(ma, mb);
  __m128i lo = _mm_mullo_epi16(ma, mb);
  ma = _mm_unpackhi_epi16(lo, hi);
  mb = _mm_unpacklo_epi16(lo, hi);
  __m128i sums = _mm_add_epi32(ma, mb);
  ap += 8;
  bp += 8;
  for (int i = 1; i < num_taps; i++) {
    ma = _mm_load_si128((const __m128i *) ap);
    mb = _mm_load_si128((const __m128i *) bp);
    hi = _mm_mulhi_epi16(ma, mb);
    lo = _mm_mullo_epi16(ma, mb);
    ma = _mm_unpackhi_epi16(lo, hi);
    mb = _mm_unpacklo_epi16(lo, hi);
    sums = _mm_add_epi32(sums, ma);
    sums = _mm_add_epi32(sums, mb);
    ap += 8;
    bp += 8;
  }
  tmp = _mm_shuffle_epi32(sums, _MM_SHUFFLE(0, 1, 2, 3));
  sums = _mm_add_epi32(sums, tmp);
  tmp = _mm_shuffle_epi32(sums, _MM_SHUFFLE(1, 0, 0, 1));
  sums = _mm_add_epi32(sums, tmp);
  return _mm_cvtsi128_si32(sums);
  //  int check = 0;
  //  for(int i=0; i<num_taps * 8; i++)
  //    check += a[i] * b[i];
  //  assert(check == result);
#else
  int result = 0;
  for(int i=0; i<num_taps * 8; i++) {
    result += a[i] * b[i];
  }
  return result;
#endif
}

#ifdef DEBUG_SUBPIXEL_MATCHING
static int
bot_pgm_write_fname(const char *fname, const uint8_t * pixels,
    int width, int height, int rowstride)
{
  FILE *fp = fopen(fname, "wb");
  if (!fp) return -1;
  fprintf(fp, "P5\n%d\n%d\n%d\n", width, height, 255);
  int i, count;
  for (i=0; i<height; i++) {
    count = fwrite(pixels + i*rowstride, width, 1, fp);
    if (1 != count) return -1;
  }
  fclose(fp);
  return 0;
}
#endif


// sub-pixel refinement of feature matches, for more accurate depth
// estimation.
void refineFeatureMatch(PyramidLevel* ref_level,
                        PyramidLevel* target_level,
                        Eigen::Vector2d ref_uv,
                        Eigen::Vector2d init_target_uv,
                        Eigen::Vector2d * final_target_uv,
                        float *delta_sse)
{
  // get the reference descriptor
  const uint8_t* ref_gray = ref_level->getGrayscaleImage();
  int ref_gray_stride = ref_level->getGrayscaleImageStride();

  int desc_len = ref_level->getDescriptorLength();
  int desc_stride = ref_level->getDescriptorStride();

  uint8_t ref_descriptor[desc_stride] __attribute__ ((aligned (16)));
  ref_level->populateDescriptorInterp(ref_uv.x(), ref_uv.y(), ref_descriptor);

  int buf_num_bytes = round_up_to_multiple(desc_len * sizeof(int16_t), 16);
  int buf_num_elements = buf_num_bytes / sizeof(int16_t);
  int buf_num_pad_bytes = (buf_num_elements - desc_len) * sizeof(int16_t);

  // how many SSE operations does it take to compute a dot product?  Each
  // operation works on 8 elements at a time.
  int dot_prod_num_taps = buf_num_elements / 8;

  // initialize the target descriptor
  uint8_t orig_target_desc[desc_stride] __attribute__ ((aligned (16)));
  target_level->populateDescriptorInterp(init_target_uv.x(), init_target_uv.y(),
                                         orig_target_desc);
  uint8_t tgt_desc[desc_stride] __attribute__ ((aligned (16)));
  memcpy(tgt_desc, orig_target_desc, desc_stride);

  float tx = init_target_uv.x();
  float ty = init_target_uv.y();

  const uint8_t* target_gray = target_level->getGrayscaleImage();
  int tgt_gray_stride = target_level->getGrayscaleImageStride();

  int16_t pix_errs[buf_num_elements] __attribute__ ((aligned (16)));
  memset(pix_errs + desc_len, 0, buf_num_pad_bytes);

  // compute an initial error
  for (int i = 0; i < desc_len; i++) {
    pix_errs[i] = tgt_desc[i] - ref_descriptor[i];
  }
  int initial_sse = dot_int16_aligned(pix_errs, pix_errs, dot_prod_num_taps);

  int32_t final_sse = initial_sse;

  // Minimization using Efficient Second-order Minimization (ESM) method
  // described in:
  //   Selim Benhimane and Ezio Malis, "Real-time image-based tracking of
  //   planes using Efficient Second-order Minimization", IROS 2004

  // compute reference image gradients
  int16_t ref_desc_dx[buf_num_elements] __attribute__ ((aligned (16)));
  int16_t ref_desc_dy[buf_num_elements] __attribute__ ((aligned (16)));
  memset(ref_desc_dx + desc_len, 0, buf_num_pad_bytes);
  memset(ref_desc_dy + desc_len, 0, buf_num_pad_bytes);
  int rdesc_offset = ref_uv.y() * ref_gray_stride + ref_uv.x();
  const int* ref_desc_offsets = ref_level->getDescriptorIndexOffsets();
  for (int i = 0; i < desc_len; i++) {
    int k = rdesc_offset + ref_desc_offsets[i];
    ref_desc_dx[i] = ref_gray[k + 1] - ref_gray[k];
    ref_desc_dy[i] = ref_gray[k + ref_gray_stride] - ref_gray[k];
    assert(k + ref_gray_stride < ref_level->getHeight() * ref_gray_stride);
  }

  int16_t Mx[buf_num_elements] __attribute__ ((aligned (16)));
  int16_t My[buf_num_elements] __attribute__ ((aligned (16)));
  memset(Mx + desc_len, 0, buf_num_pad_bytes);
  memset(My + desc_len, 0, buf_num_pad_bytes);

  int max_iterations = 6;
  const int* tgt_desc_offsets = target_level->getDescriptorIndexOffsets();
  for (int iter_num = 0; iter_num < max_iterations; iter_num++) {
    // compute target image gradients at current position and
    // M = sum of Jacobians at reference and current positions
    int tdesc_center = ((int) ty) * tgt_gray_stride + (int) tx;
    for (int i = 0; i < desc_len; i++) {
      int k = tdesc_center + tgt_desc_offsets[i];
      Mx[i] = ref_desc_dx[i] + (target_gray[k + 1] - target_gray[k]);
      My[i] = ref_desc_dy[i] + (target_gray[k + tgt_gray_stride] - target_gray[k]);
    }

    // S = M'*M
    double S_00 = dot_int16_aligned(Mx, Mx, dot_prod_num_taps);
    double S_01 = dot_int16_aligned(Mx, My, dot_prod_num_taps);
    double S_11 = dot_int16_aligned(My, My, dot_prod_num_taps);

    // S^{-1}
    double det = S_00 * S_11 - S_01 * S_01;
    if (det <= 1e-9)
      break;
    double Sinv_00 = S_11 / det;
    double Sinv_01 = -S_01 / det;
    double Sinv_11 = S_00 / det;

    // M'*err
    double gx = dot_int16_aligned(Mx, pix_errs, dot_prod_num_taps);
    double gy = dot_int16_aligned(My, pix_errs, dot_prod_num_taps);

    // M^{+} = pseudoinverse of M
    // delta = -2 * M^{+} * err
    double delta_x = -2 * (Sinv_00 * gx + Sinv_01 * gy);
    double delta_y = -2 * (Sinv_01 * gx + Sinv_11 * gy);

    float next_tx = tx + delta_x;
    float next_ty = ty + delta_y;

    // stop if the keypoint is about to go out of bounds
    if (!target_level->isLegalKeypointCoordinate(next_tx, next_ty))
      break;

    // compute shifted target descriptor and error
    target_level->populateDescriptorInterp(next_tx, next_ty, tgt_desc);

    for (int i = 0; i < desc_len; i++)
      pix_errs[i] = tgt_desc[i] - ref_descriptor[i];
    int sse = dot_int16_aligned(pix_errs, pix_errs, dot_prod_num_taps);

#ifdef DEBUG_SUBPIXEL_MATCHING
    printf("sse: %d, delta : %f, %f\n", sse, delta_x, delta_y);
    char tfname[80];
    sprintf(tfname, "tgt-desc-%d.pgm", iter_num);
    bot_pgm_write_fname(tfname, tgt_desc, fwsize, fwsize, fwsize);
#endif

    if (sse < final_sse) {
      // only step if the error decreases.
      tx = next_tx;
      ty = next_ty;
      final_sse = sse;
    } else {
      // if stepping would increase the error, then just give up.
      // TODO modify damping parameters instead?  Might not be worth it
#ifdef DEBUG_SUBPIXEL_MATCHING
      printf("  Error is increasing! Abort optimization.\n");
#endif
      break;
    }

    // stop if we're not moving that much
    if (fabs(delta_x) < 0.1 && fabs(delta_y) < 0.1)
      break;
  }

  *final_target_uv = Eigen::Vector2d(tx, ty);
  *delta_sse = final_sse - initial_sse;
}

} /*  */
