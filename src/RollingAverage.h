#ifndef Carver_RollingAverage_h
#define Carver_RollingAverage_h

class RollingAverage
{
public:
    std::vector<float> samples;
    float total;
    int numSamples;
    
    float Average()
    {
        if (samples.size() == 0)
        {
            return 0;
        }
        return total / (float)samples.size();
    }
    
    RollingAverage()
    {
        RollingAverage(10);
    }
    
    RollingAverage(int numSamples_)
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
};

#endif
