#include "peak_detector.h"
#include <numeric>
#include <cmath>
#include <algorithm>


double mean(std::vector<double>& v) {
  double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
  return sum / v.size();
}

double stdDev(std::vector<double>& v, double m) {
  double accum = 0.0;
  std::for_each (std::begin(v), std::end(v), [&](const double d) {
    accum += (d - m) * (d - m);
  });
  return sqrt(accum / (v.size()-1));
}

// from https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data thread
std::vector<int> smoothedZScore(std::vector<double>& input)
{
    //lag for the smoothing functions
    unsigned int lag = 150;
    //number of  standard deviations for signal
    double threshold = 2.5;
    //between 0 and 1, where 1 is normal influence, 0.5 is half
    double influence = 0.0025;

    if (input.size() <= lag + 2)
    {
        std::vector<int> emptyVec;
        return emptyVec;
    }

    //Initialise variables
    std::vector<int> signals(input.size(), 0.0);
    std::vector<double> filteredY(input.size(), 0.0);
    std::vector<double> avgFilter(input.size(), 0.0);
    std::vector<double> stdFilter(input.size(), 0.0);
    std::vector<double> subVecStart(input.begin(), input.begin() + lag);
    double m = mean(subVecStart);
    avgFilter[lag] = m;
    stdFilter[lag] = stdDev(subVecStart, m);

    for (size_t i = lag + 1; i < input.size(); i++)
    {
        if (std::abs(input[i] - avgFilter[i - 1]) > threshold * stdFilter[i - 1])
        {
            if (input[i] > avgFilter[i - 1])
            {
                signals[i] = 1; //# Positive signal
            }
            else
            {
                signals[i] = -1; //# Negative signal
            }
            //Make influence lower
            filteredY[i] = influence* input[i] + (1 - influence) * filteredY[i - 1];
        }
        else
        {
            signals[i] = 0; //# No signal
            filteredY[i] = input[i];
        }
        //Adjust the filters
        std::vector<double> subVec(filteredY.begin() + i - lag, filteredY.begin() + i);
        double m = mean(subVec);
        avgFilter[i] = m;
        stdFilter[i] = stdDev(subVec, m);
    }
    return signals;
}
