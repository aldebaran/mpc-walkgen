#ifndef PORTABILITY_GETTIMEOFDAY_HH
# define PORTABILITY_GETTIMEOFDAY_HH

// This deals with gettimeofday portability issues.
# ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  ifdef WIN32
#   include <time.h>

struct timezone
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

int gettimeofday(struct timeval *tv, struct timezone *tz);

#  else // WIN32
#   error "gettimeof day does not seem to be supported on your platform."
#  endif // WIN32
# endif //! HAVE_SYS_TIME_H
#endif //! PORTABILITY_GETTIMEOFDAY_HH
