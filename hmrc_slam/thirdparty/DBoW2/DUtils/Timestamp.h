/*
 * File: Timestamp.h
 * Author: Dorian Galvez-Lopez
 * Date: March 2009
 * Description: timestamping functions
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_TIMESTAMP__
#define __D_TIMESTAMP__

#include <iostream>
using namespace std;

namespace DUtils {

/// Timestamp
class Timestamp
{
public:

  /// Options to initiate a timestamp
  enum tOptions
  {
    NONE = 0,
    CURRENT_TIME = 0x1,
    ZERO = 0x2
  };
  
public:
  
  	Timestamp(Timestamp::tOptions option = NONE);
	
		virtual ~Timestamp(void);

    bool empty() const;

		void setToCurrentTime();

		inline void setTime(unsigned long secs, unsigned long usecs){
		m_secs = secs;
		m_usecs = usecs;
	}
	
		inline void getTime(unsigned long &secs, unsigned long &usecs) const
	{
	  secs = m_secs;
	  usecs = m_usecs;
	}

		void setTime(const string &stime);
	
		void setTime(double s);
	
		double getFloatTime() const;

		string getStringTime() const;

		double operator- (const Timestamp &t) const;

		Timestamp plus(unsigned long s, unsigned long us) const;

    Timestamp minus(unsigned long s, unsigned long us) const;

    Timestamp& operator+= (double s);
  
    Timestamp& operator-= (double s);

		Timestamp operator+ (double s) const;

		Timestamp operator- (double s) const;

		bool operator> (const Timestamp &t) const;

		bool operator>= (const Timestamp &t) const;

		bool operator== (const Timestamp &t) const;

		bool operator< (const Timestamp &t) const;

		bool operator<= (const Timestamp &t) const;

    string Format(bool machine_friendly = false) const;

		static string Format(double s);
	

protected:
  /// Seconds
	unsigned long m_secs;	// seconds
	/// Microseconds
	unsigned long m_usecs;	// microseconds
};

}

#endif

