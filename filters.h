#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

// --------------------- Low-Pass Filter ---------------------
class LowPassFilter {
  float alpha, prev_value;
  bool initialized;

public:
  LowPassFilter(float cutoff, float sampling_frequency) {
    float dt = 1.0 / sampling_frequency;
    float RC = 1.0 / (2 * 3.1416 * cutoff);
    alpha = dt / (RC + dt);
    initialized = false;
  }

  float process(float value) {
    if (!initialized) {
      prev_value = value;
      initialized = true;
    }
    prev_value = alpha * value + (1.0 - alpha) * prev_value;
    return prev_value;
  }

  void reset() { initialized = false; }
};

// --------------------- High-Pass Filter ---------------------
class HighPassFilter {
  float alpha, prev_input, prev_output;
  bool initialized;

public:
  HighPassFilter(float cutoff, float sampling_frequency) {
    float dt = 1.0 / sampling_frequency;
    float RC = 1.0 / (2 * 3.1416 * cutoff);
    alpha = RC / (RC + dt);
    initialized = false;
  }

  float process(float value) {
    if (!initialized) {
      prev_input = value;
      prev_output = 0;
      initialized = true;
    }
    float output = alpha * (prev_output + value - prev_input);
    prev_input = value;
    prev_output = output;
    return output;
  }

  void reset() { initialized = false; }
};

// --------------------- Differentiator ---------------------
class Differentiator {
  float prev_value;
  float sampling_frequency;
  bool initialized;

public:
  Differentiator(float freq) {
    sampling_frequency = freq;
    initialized = false;
  }

  float process(float value) {
    if (!initialized) {
      prev_value = value;
      initialized = true;
      return NAN;
    }
    float diff = (value - prev_value) * sampling_frequency;
    prev_value = value;
    return diff;
  }

  void reset() { initialized = false; }
};

// --------------------- Moving Average Filter ---------------------
template <int size>
class MovingAverageFilter {
  float values[size];
  int index, count_total;
  float sum;

public:
  MovingAverageFilter() { reset(); }

  int count() const { return count_total; }

  void reset() {
    for (int i = 0; i < size; i++) values[i] = 0;
    index = 0;
    count_total = 0;
    sum = 0;
  }

  float process(float value) {
    sum -= values[index];
    values[index] = value;
    sum += value;

    index = (index + 1) % size;
    if (count_total < size) count_total++;
    return sum / count_total;
  }
};

// --------------------- Min Max Average Statistic ---------------------
class MinMaxAvgStatistic {
  float min_val, max_val, sum;
  int count;
  bool initialized;

public:
  MinMaxAvgStatistic() { reset(); }

  void reset() {
    min_val = max_val = sum = 0;
    count = 0;
    initialized = false;
  }

  void process(float value) {
    if (!initialized) {
      min_val = max_val = value;
      initialized = true;
    }
    if (value < min_val) min_val = value;
    if (value > max_val) max_val = value;
    sum += value;
    count++;
  }

  float minimum() const { return initialized ? min_val : NAN; }
  float maximum() const { return initialized ? max_val : NAN; }
  float average() const { return (count > 0) ? (sum / count) : NAN; }
};

#endif
