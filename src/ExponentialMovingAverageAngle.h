//
//  ExponentialMovingAverageAngle.h
//  Carver
//
//  Created by paulobala on 12/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_ExponentialMovingAverageAngle_h
#define Carver_ExponentialMovingAverageAngle_h

#include "RollingAverage.h"

class ExponentialMovingAverageAngle
{
    std::vector<float> samples;
    float total;
    float totalSin;
    float totalCos;
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
    
    float ExponentialAverage(){
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
    
    ExponentialMovingAverageAngle(){
        RollingAverage(10);
    }
    
    ExponentialMovingAverageAngle(int numSamples_)
    {
        if (numSamples_ <= 0)
        {
            numSamples_ = 1;
        }
        
        samples = std::vector<float>();
        numSamples = numSamples_;
        total = 0;
        totalCos = 0;
        totalSin = 0;
    }
    
    void addSample(float value)
    {
        if (samples.size() == numSamples)
        {
            total -= samples[0];
            totalCos -= cos(samples[0]);
            totalSin -= sin(samples[0]);
            samples.erase(samples.begin());
        }
        
        samples.push_back(value);
        total += value;
        totalCos += cos(value);
        totalSin += sin(value);
    }
    
    
    void clearSamples()
    {
        total = 0;
        totalCos = 0;
        totalSin = 0;
        samples.clear();
    }
    
    
    void initializeSamples(float v)
    {
        for(int i = 0; i < numSamples; i++){
            samples.push_back(v);
        }
        total = v * numSamples;
        totalCos = cos(v) * numSamples;
        totalSin = sin(v) * numSamples;
    }
    
    float lastValue(){
        if(samples.size() >0){return samples[samples.size()-1];}
        else
            return 0;
    }
    
    float getAngle(){
        return atan2(totalSin,totalCos)
        ;
    }
};

#endif
