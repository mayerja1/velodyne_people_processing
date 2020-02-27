#ifndef PEAK_DETECTOR_H
#define PEAK_DETECTOR_H
#include <vector>
double mean(std::vector<double>& v);

double stdDev(std::vector<double>& v, double m);

std::vector<int> smoothedZScore(std::vector<double>& input);
#endif /* end of include guard: PEAK_DETECTOR_H */
