#define _GNU_SOURCE
#include <dlfcn.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
// Test only: inject a 1.6s disk wait on alternating status snapshots.
int fsync(int fd) {
  static int (*real_sync)(int)=0;
  static unsigned status_calls=0;
  if (!real_sync) real_sync=dlsym(RTLD_NEXT,"fsync");
  char link[80],path[4096];snprintf(link,sizeof(link),"/proc/self/fd/%d",fd);
  ssize_t n=readlink(link,path,sizeof(path)-1);
  if(n>0) {
    path[n]=0;
    if(strstr(path,"/state.json.") && (++status_calls%2)==0) {
      struct timespec delay={1,600000000};nanosleep(&delay,0);
    }
  }
  return real_sync(fd);
}
