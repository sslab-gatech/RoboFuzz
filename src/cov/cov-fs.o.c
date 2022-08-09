// trace-pc-guard-cb.cc
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sanitizer/coverage_interface.h>

const char* covfile_path;

// This callback is inserted by the compiler as a module constructor
// into every DSO. 'start' and 'stop' correspond to the
// beginning and end of the section with the guards for the entire
// binary (executable or DSO). The callback will be called at least
// once per DSO and may be called multiple times with the same parameters.
void __sanitizer_cov_trace_pc_guard_init(uint32_t *start,
                                                    uint32_t *stop) {
  static uint64_t N;  // Counter for the guards.
  if (start == stop || *start) return;  // Initialize only once.
  printf("INIT: %p %p\n", start, stop);
  for (uint32_t *x = start; x < stop; x++)
    *x = ++N;  // Guards should start from 1.

  covfile_path = getenv("COVFILE_PATH");
  if (!covfile_path) {
    printf("COVFILE_PATH envvar is not set. Defaulting to /tmp/cov\n");
    covfile_path = "/tmp/cov";
  }
  int ret = unlink(covfile_path);
  if (ret < 0) {
    printf("[-] could not remove %s\n", covfile_path);
  }
}

// This callback is inserted by the compiler on every edge in the
// control flow (some optimizations apply).
// Typically, the compiler will emit the code like this:
//    if(*guard)
//      __sanitizer_cov_trace_pc_guard(guard);
// But for large functions it will emit a simple call:
//    __sanitizer_cov_trace_pc_guard(guard);
void __sanitizer_cov_trace_pc_guard(uint32_t *guard) {
  if (!*guard) return;  // Duplicate the guard check.
  // If you set *guard to 0 this code will not be called again for this edge.
  // Now you can get the PC and do whatever you want:
  //   store it somewhere or symbolize it and print right away.
  // The values of `*guard` are as you set them in
  // __sanitizer_cov_trace_pc_guard_init and so you can make them consecutive
  // and use them to dereference an array or a bit vector.
  void *PC = __builtin_return_address(0);
  /* char PcDescr[1024]; */
  // This function is a part of the sanitizer run-time.
  // To use it, link with AddressSanitizer or other sanitizer.

  char buf[20] = {0, };
  int numbytes = snprintf(buf, 20, "%lx ", (unsigned long)PC);
  /* __sanitizer_symbolize_pc(PC, "%p %F %L", PcDescr, sizeof(PcDescr)); */
  int fd = open(covfile_path, O_CREAT | O_RDWR | O_APPEND, 0644);
  ssize_t size = write(fd, buf, numbytes);
  /* printf("guard: %p %x \n", guard, *guard); */
  int ret = close(fd);
}
