/*	
 * File: Random.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010, November 2011
 * Description: manages pseudo-random numbers
 * License: see the LICENSE.txt file
 *
 */

#pragma once
#ifndef __D_RANDOM__
#define __D_RANDOM__

#include <cstdlib>
#include <vector>

namespace DUtils {

/// Functions to generate pseudo-random numbers
class Random
{
public:
  class UnrepeatedRandomizer;
  
public:
		static void SeedRand();
	
		static void SeedRandOnce();

		static void SeedRand(int seed);

		static void SeedRandOnce(int seed);

		template <class T>
	static T RandomValue(){
		return (T)rand()/(T)RAND_MAX;
	}

		template <class T>
	static T RandomValue(T min, T max){
		return Random::RandomValue<T>() * (max - min) + min;
	}

		static int RandomInt(int min, int max);
	
		template <class T>
	static T RandomGaussianValue(T mean, T sigma)
	{
    // Box-Muller transformation
    T x1, x2, w, y1;

    do {
      x1 = (T)2. * RandomValue<T>() - (T)1.;
      x2 = (T)2. * RandomValue<T>() - (T)1.;
      w = x1 * x1 + x2 * x2;
    } while ( w >= (T)1. || w == (T)0. );

    w = sqrt( ((T)-2.0 * log( w ) ) / w );
    y1 = x1 * w;

    return( mean + y1 * sigma );
	}

private:

  /// If SeedRandOnce() or SeedRandOnce(int) have already been called
  static bool m_already_seeded;
  
};

// ---------------------------------------------------------------------------

/// Provides pseudo-random numbers with no repetitions
class Random::UnrepeatedRandomizer
{
public:

    UnrepeatedRandomizer(int min, int max);
  ~UnrepeatedRandomizer(){}
  
    UnrepeatedRandomizer(const UnrepeatedRandomizer& rnd);
  
    UnrepeatedRandomizer& operator=(const UnrepeatedRandomizer& rnd);
  
    int get();
  
    inline bool empty() const { return m_values.empty(); }
  
    inline unsigned int left() const { return m_values.size(); }
  
    void reset();
  
protected:

    void createValues();

protected:

  /// Min of range of values
  int m_min;
  /// Max of range of values
  int m_max;

  /// Available values
  std::vector<int> m_values;

};

}

#endif

