#ifndef IIRFILTER_H
#define IIRFILTER_H

template <int M, int N>
class IIRFilter {
public:
  IIRFilter(const float* a, const float* b)
  {
    // Copy in the coefficients and initialize the feedback values to zero.
    for (int i=0; i<N; ++i) {
      _a[i] = a[i];
      _y[i] = 0.0;
    }
    for (int i=0; i<M; ++i) {
      _b[i] = b[i];
      _x[i] = 0.0;
    }
  }

  // Apply the filter to the new input value x.  Will return a filtered value y.
  float filter(float x) {
    // Shift the previous inputs forward by one position to make room for new input.
    for (int i=1; i<M; ++i) {
      _x[i] = _x[i-1];
    }
    _x[0] = x;  // Add most recent input.
    // Shift the previous outputs forward by one position to make room for new output too.
    for (int i=1; i<N; ++i) {
      _y[i] = _y[i-1];
    }
    // Compute new output value.  This uses the IIR equation from scipy's signal filter function here:
    //   https://docs.scipy.org/doc/scipy-0.19.0/reference/generated/scipy.signal.lfilter.html#scipy.signal.lfilter
    float y = 0.0;
    // First compute sum of all previous inputs scaled by their coefficients.
    for (int i=0; i<M; ++i) {
      y += _b[i]*_x[i];
    }
    // Now subtract sum of all previous outputs scaled by their coefficients.
    // Ignore the first output coefficient as its a value which scales the final result.
    for (int i=1; i<N; ++i) {
      y -= _a[i]*_y[i];
    }
    // Finally scale by a[0] coefficient.
    y /= _a[0];
    // Save output result and return it.
    _y[0] = y;
    return y;
  }

private:
  float _a[N];
  float _b[M];
  float _x[M];
  float _y[N];
};

#endif
