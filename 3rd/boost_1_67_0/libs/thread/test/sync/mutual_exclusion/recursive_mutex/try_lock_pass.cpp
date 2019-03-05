//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// Copyright (C) 2011 Vicente J. Botet Escriba
//
//  Distributed under the Boost Software License, Version 1.0. (See accompanying
//  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

// <boost/thread/recursive_mutex.hpp>

// class recursive_mutex;

// bool try_lock();

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/detail/lightweight_test.hpp>
#include <iostream>

boost::recursive_mutex m;

#if defined BOOST_THREAD_USES_CHRONO
typedef boost::chrono::system_clock Clock;
typedef Clock::time_point time_point;
typedef Clock::duration duration;
typedef boost::chrono::milliseconds ms;
typedef boost::chrono::nanoseconds ns;
#else
#endif

#ifdef BOOST_THREAD_PLATFORM_WIN32
const ms max_diff(250);
#else
const ms max_diff(75);
#endif

void f()
{
#if defined BOOST_THREAD_USES_CHRONO
  time_point t0 = Clock::now();
//  BOOST_TEST(!m.try_lock());
//  BOOST_TEST(!m.try_lock());
//  BOOST_TEST(!m.try_lock());
  while (!m.try_lock())
    ;
  time_point t1 = Clock::now();
  BOOST_TEST(m.try_lock());
  m.unlock();
  m.unlock();
  ns d = t1 - t0 - ms(250);
  std::cout << "diff= " << d.count() << std::endl;
  std::cout << "max_diff= " << max_diff.count() << std::endl;
  BOOST_TEST(d < max_diff);
#else
  //time_point t0 = Clock::now();
  //BOOST_TEST(!m.try_lock());
  //BOOST_TEST(!m.try_lock());
  //BOOST_TEST(!m.try_lock());
  while (!m.try_lock())
    ;
  //time_point t1 = Clock::now();
  BOOST_TEST(m.try_lock());
  m.unlock();
  m.unlock();
  //ns d = t1 - t0 - ms(250);
  //BOOST_TEST(d < max_diff);
#endif
}

int main()
{
  m.lock();
  boost::thread t(f);
#if defined BOOST_THREAD_USES_CHRONO
  boost::this_thread::sleep_for(ms(250));
#else
#endif
  m.unlock();
  t.join();

  return boost::report_errors();
}


