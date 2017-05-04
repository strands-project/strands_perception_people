#!/usr/bin/env python

import numpy as np
import cv2
import scipy.ndimage
import scipy.interpolate




def laser_angles(N, fov):
    return np.linspace(-fov*0.5, fov*0.5, N)


def rphi_to_xy(r, phi):
    return r * -np.sin(phi), r * np.cos(phi)

def xy_to_rphi(x, y):
    # NOTE: Axes rotated by 90 CCW by intent, so tat 0 is top.
    return np.hypot(x, y), np.arctan2(-x, y)


def generate_cut_outs(scan, standard_depth=4.0, window_size=48, threshold_distance=1.0, npts=None, center=True, border=29.99, resample_type='cv', **kw):
    '''
    Generate window cut outs that all have a fixed size independent of depth.
    This means areas close to the scanner will be subsampled and areas far away
    will be upsampled.
    All cut outs will have values between `-threshold_distance` and `+threshold_distance`
    as they are normalized by the center point.

    - `scan` an iterable of radii within a laser scan.
    - `standard_depth` the reference distance (in meters) at which a window with `window_size` gets cut out.
    - `window_size` the window of laser rays that will be extracted everywhere.
    - `npts` is the number of final samples to have per window. `None` means same as `window_size`.
    - `threshold_distance` the distance in meters from the center point that will be used to clamp the laser radii.
      Since we're talking about laser-radii, this means the cutout is a donut-shaped hull, as opposed to a rectangular hull.
      This can be `np.inf` to skip the clamping altogether.
    - `center` whether to center the cutout around the current laser point's depth (True), or keep depth values raw (False).
    - `border` the radius value to fill the half of the outermost windows with.
    - `resample_type` specifies the resampling API to be used. Possible values are:
        - `cv` for OpenCV's `cv2.resize` function using LINEAR/AREA interpolation.
        - `zoom` for SciPy's `zoom` function, to which options such as `order=3` can be passed as extra kwargs.
        - `int1d` for SciPy's `interp1d` function, to which options such as `kind=3` can be passed as extra kwargs.
    '''
    s_np = np.fromiter(iter(scan), dtype=np.float32)
    N = len(s_np)

    npts = npts or window_size
    cut_outs = np.zeros((N, npts), dtype=np.float32)

    current_size = (window_size * standard_depth / s_np).astype(np.int32)
    start = -current_size//2 + np.arange(N)
    end = start + current_size
    s_np_extended = np.append(s_np, border)

    # While we don't really need to special-case, it should save precious computation.
    if threshold_distance != np.inf:
        near = s_np-threshold_distance
        far  = s_np+threshold_distance

    for i in range(N):
        # Get the window.
        sample_points = np.arange(start[i], end[i])
        sample_points[sample_points < 0] = -1
        sample_points[sample_points >= N] = -1
        window = s_np_extended[sample_points]

        # Threshold the near and far values, then
        if threshold_distance != np.inf:
            window = np.clip(window, near[i], far[i])

        # shift everything to be centered around the middle point.
        if center:
            window -= s_np[i]

        # Values will now span [-d,d] if `center` and `clamp` are both True.

        # resample it to the correct size.
        if resample_type == 'cv':
            # Use 'INTER_LINEAR' for when down-sampling the image LINEAR is ridiculous.
            # It's just 1ms slower for a whole scan in the worst case.
            interp = cv2.INTER_AREA if npts < len(window) else cv2.INTER_LINEAR
            cut_outs[i,:] = cv2.resize(window[None], (npts,1), interpolation=interp)[0]
        elif resample_type == 'zoom':
            scipy.ndimage.interpolation.zoom(window, npts/len(window), output=cut_outs[i,:], **kw)
        elif resample_type == 'int1d':
            cut_outs[i,:] = scipy.interpolate.interp1d(np.linspace(0,1, num=len(window), endpoint=True), window, assume_sorted=True, copy=False, **kw)(np.linspace(0,1,num=npts, endpoint=True))

    return cut_outs


def win2global(r, phi, dx, dy):
    y = r + dy
    dphi = np.arctan2(dx, y)  # dx first is correct due to problem geometry dx -> y axis and vice versa.
    return y/np.cos(dphi), phi + dphi

def pred2votes(scan, y_conf, y_offs, thresh, fov):
    locs, probs = [], []
    for (pno, pwc, pwa), (dx, dy), r, phi in zip(y_conf, y_offs, scan, laser_angles(len(scan), fov)):
        if thresh < pwc+pwa:
            locs.append(rphi_to_xy(*win2global(r, phi, dx, dy)))
            probs.append((pwc, pwa))
    return locs, probs


def votes_to_detections(locations, probas=None, in_rphi=True, out_rphi=True, bin_size=0.1, blur_win=21, blur_sigma=2.0, x_min=-15.0, x_max=15.0, y_min=-5.0, y_max=15.0, retgrid=False):
    '''
    Convert a list of votes to a list of detections based on Non-Max supression.

    - `locations` an iterable containing predicted x/y or r/phi pairs.
    - `probas` an iterable containing predicted probabilities. Considered all ones if `None`.
    - `in_rphi` whether `locations` is r/phi (True) or x/y (False).
    - `out_rphi` whether the output should be r/phi (True) or x/y (False).
    - `bin_size` the bin size (in meters) used for the grid where votes are cast.
    - `blur_win` the window size (in bins) used to blur the voting grid.
    - `blur_sigma` the sigma used to compute the Gaussian in the blur window.
    - `x_min` the left limit for the voting grid, in meters.
    - `x_max` the right limit for the voting grid, in meters.
    - `y_min` the bottom limit for the voting grid in meters.
    - `y_max` the top limit for the voting grid in meters.

    Returns a list of tuples (x,y,class) or (r,phi,class) where `class` is
    the index into `probas` which was highest for each detection, thus starts at 0.

    NOTE/TODO: We really should replace `bin_size` by `nbins` so as to avoid "remainders".
               Right now, we simply ignore the remainder on the "max" side.
    '''
    locations = np.array(locations)
    if len(locations) == 0:
        return []

    if probas is None:
        probas = np.ones((len(locations),1))
    else:
        probas = np.array(probas)
        assert len(probas) == len(locations) and probas.ndim == 2, "Invalid format of `probas`"

    x_range = int((x_max-x_min)/bin_size)
    y_range = int((y_max-y_min)/bin_size)
    grid = np.zeros((x_range, y_range, 1+probas.shape[1]), np.float32)

    # Update x/y max to correspond to the end of the last bin.
    # TODO: fix this as stated in the docstring.
    x_max = x_min + x_range*bin_size
    y_max = y_min + y_range*bin_size

    # Do the voting into the grid.
    for loc, p in zip(locations, probas):
        x,y = rphi_to_xy(*loc) if in_rphi else loc

        # Skip votes outside the grid.
        if not (x_min < x < x_max and y_min < y < y_max):
            continue

        x = int((x-x_min)/bin_size)
        y = int((y-y_min)/bin_size)
        grid[x,y,0] += np.sum(p)
        grid[x,y,1:] += p

    # Yes, this blurs each channel individually, just what we need!
    grid = cv2.GaussianBlur(grid, (blur_win,blur_win), blur_sigma)

    # Find the maxima (NMS) only in the "common" voting grid.
    grid_all = grid[:,:,0]
    max_grid = scipy.ndimage.maximum_filter(grid_all, size=3)
    maxima = (grid_all == max_grid) & (grid_all != 0)
    m_x, m_y = np.where(maxima)

    # Probabilities of all classes where maxima were found.
    m_p = grid[m_x, m_y, 1:]

    # Back from grid-bins to real-world locations.
    m_x = m_x*bin_size + x_min + bin_size/2
    m_y = m_y*bin_size + y_min + bin_size/2
    maxima = [(xy_to_rphi(x,y) if out_rphi else (x,y)) + (np.argmax(p),) for x,y,p in zip(m_x, m_y, m_p)]
    return (maxima, grid) if retgrid else maxima


def pred2det_comb(scan, conf, offs, thresh, fov, out_rphi=False, **kw):
    dets = [[] for _ in range(conf.shape[-1])]

    locs, probas = pred2votes(scan, conf, offs, thresh=thresh, fov=fov)
    if 0 < len(locs):
        for x, y, lbl in votes_to_detections(locs, probas, in_rphi=False, out_rphi=out_rphi, **kw):
            dets[0].append((x,y))
            dets[lbl+1].append((x,y))

    return dets