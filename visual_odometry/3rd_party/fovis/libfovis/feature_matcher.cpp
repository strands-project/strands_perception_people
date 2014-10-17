#include "feature_matcher.hpp"

#include "pyramid_level.hpp"
#include "internal_utils.hpp"
#include "sad.hpp"

namespace fovis
{

FeatureMatcher::FeatureMatcher() :
  _ref_feature_capacity(500),
  _target_feature_capacity(500)
{
  _ref_to_target_indices = (int32_t*) malloc(_ref_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _ref_to_target_scores = (int32_t*) malloc(_ref_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _target_to_ref_indices = (int32_t*) malloc(_target_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _target_to_ref_scores = (int32_t*) malloc(_target_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
}

FeatureMatcher::~FeatureMatcher()
{
  free(_ref_to_target_indices);
  free(_ref_to_target_scores);
  free(_target_to_ref_indices);
  free(_target_to_ref_scores);
}

void
FeatureMatcher::matchFeatures(PyramidLevel* ref_level,
                              PyramidLevel* target_level,
                              const std::vector<std::vector<int> >& candidates,
                              FeatureMatch* matches,
                              int* num_matches)
{
  int num_ref_features = ref_level->getNumKeypoints();
  int num_target_features = target_level->getNumKeypoints();

  // allocate more space for buffers if necessary
  if (num_ref_features > _ref_feature_capacity) {
    _ref_to_target_indices = (int32_t*) realloc(_ref_to_target_indices, num_ref_features * sizeof(int32_t));
    _ref_to_target_scores = (int32_t*) realloc(_ref_to_target_scores, num_ref_features * sizeof(int32_t));
    _ref_feature_capacity = num_ref_features;
  }

  if (num_target_features > _target_feature_capacity) {
    int buf_size = num_target_features * sizeof(int32_t);
    _target_to_ref_indices = (int32_t*) realloc(_target_to_ref_indices, buf_size);
    _target_to_ref_scores = (int32_t*) realloc(_target_to_ref_scores, buf_size);
    _target_feature_capacity = num_target_features;
  }

  int descriptor_len = ref_level->getDescriptorLength();
  assert(descriptor_len == target_level->getDescriptorLength());

  SAD sad(descriptor_len);

  int32_t worst_score = sad.getWorstScore();

  // initialize book-keeping for feature matching
  for (int i = 0; i < num_ref_features; i++) {
    _ref_to_target_scores[i] = worst_score + 1;
    _ref_to_target_indices[i] = -1;
  }
  for (int i = 0; i < num_target_features; i++) {
    _target_to_ref_scores[i] = worst_score + 1;
    _target_to_ref_indices[i] = -1;
  }

  // for each feature in the target frame, compute the best matching feature in
  // the reference frame.
  // Similarly, compute the best matching feature in the target frame for each
  // feature in the reference frame.
  // Match score is defined as the sum of absolute differences between two feature
  // descriptors.  Lower scores (less difference) are better.
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    const uint8_t * ref_desc = ref_level->getDescriptor(ref_ind);

    const std::vector<int>& ref_candidates(candidates[ref_ind]);
    for (std::vector<int>::const_iterator ref_candidates_itr = ref_candidates.begin(),
         ref_candidates_end = ref_candidates.end();
         ref_candidates_itr != ref_candidates_end;
         ++ref_candidates_itr) {
      int target_ind = *ref_candidates_itr;
      const uint8_t * target_desc = target_level->getDescriptor(target_ind);

      int score = sad.score(ref_desc, target_desc);
      assert(score <= worst_score);

      // see if this score is the best for either descriptor
      if (score < _ref_to_target_scores[ref_ind]) {
        _ref_to_target_scores[ref_ind] = score;
        _ref_to_target_indices[ref_ind] = target_ind;
      }
      if (score < _target_to_ref_scores[target_ind]) {
        _target_to_ref_scores[target_ind] = score;
        _target_to_ref_indices[target_ind] = ref_ind;
      }
    }
  }

  // now find features that are mutual best matches in both directions
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    int target_ind = _ref_to_target_indices[ref_ind];
    if (target_ind >= 0 &&
        _target_to_ref_indices[target_ind] == ref_ind) {

      KeypointData* ref_kpdata = ref_level->getKeypointData(ref_ind);
      KeypointData* target_kpdata = target_level->getKeypointData(target_ind);

      assert(ref_kpdata->kp.u >= 0 && ref_kpdata->kp.v >= 0 &&
             ref_kpdata->kp.u < ref_level->getWidth() &&
             ref_kpdata->kp.v < ref_level->getHeight());
      assert(target_kpdata->kp.u >= 0 && target_kpdata->kp.v >= 0 &&
             target_kpdata->kp.u < target_level->getWidth() &&
             target_kpdata->kp.v < target_level->getHeight());

      FeatureMatch match(target_kpdata, ref_kpdata);
      match.status = MATCH_OK;
      matches[*num_matches] = match;
      (*num_matches)++;
    }
  }

}

} /*  */
