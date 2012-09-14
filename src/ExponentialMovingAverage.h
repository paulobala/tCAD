
#ifndef tCAD_ExponentialMovingAverage_h
#define tCAD_ExponentialMovingAverage_h

#include "RollingAverage.h"
/*
 Exponential Moving average
 */
class ExponentialMovingAverage
{
    std::vector<float> samples;
    float total;//sum of samples
    int numSamples;
public:
    
    float Average()
    {
        if (samples.size() == 0)
        {
            return 0;
        }
        return total / (float)samples.size();
    }
    
    float ExponentialAverage()
    {
        return ExponentialAverage(0.9);
    }
    float ExponentialAverage(float baseValue)
    {
        float numerator = 0;
        float denominator = 0;
        float average = Average();
        for ( int i = 0; i < samples.size(); ++i )
        {
            numerator += samples[i] * powf( baseValue, samples.size() - i - 1 );
            denominator += powf( baseValue, samples.size() - i - 1 );
        }
        numerator += average * powf( baseValue, samples.size() );
        denominator += powf( baseValue, samples.size() );
        return numerator / (float) denominator;
    }
    
    ExponentialMovingAverage()
    {
        RollingAverage(10);
    }
    
    ExponentialMovingAverage(int numSamples_)
    {
        if (numSamples_ <= 0)
        {
            numSamples_ = 1;
        }
        samples = std::vector<float>();
        numSamples = numSamples_;
        total = 0;
    }
    
    void addSample(float value)
    {
        if (samples.size() == numSamples)
        {
            total -= samples[0];
            samples.erase(samples.begin());
        }
        samples.push_back(value);
        total += value;
    }
    
    
    void clearSamples()
    {
        total = 0;
        samples.clear();
    }
    
    
    void initializeSamples(float value)
    {
        for(int i = 0; i < numSamples; i++)
        {
            samples.push_back(value);
        }
        total = value * numSamples;
    }
    
    float lastValue()
    {
        if(samples.size() >0)
        {
            return samples[samples.size()-1];
        }
        else
            return 0;
    }
};

#endif
