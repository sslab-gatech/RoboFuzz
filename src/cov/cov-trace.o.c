#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>

char* shm;
int has_shm = 0;

#define CONST_PRIO 101
__attribute__((constructor(CONST_PRIO))) void __ros_fuzzer_init(void) {
  FILE *fp = fopen("/tmp/shmid", "r");
  size_t len;
  char* line = NULL;
  ssize_t read = getline(&line, &len, fp);
  fclose(fp);

  int shmid = atoi(line);
  /* printf("shm id: %d\n", shmid); */

  shm = (char *)shmat(shmid, 0, 0);
  if (shm == (void*)-1) {
    printf("cannot attach to shm id %d\n", shmid);
    abort();
  }
  printf("Attached ti shm id %d at: %p\n", shmid, shm);
  has_shm = 1;
}

void __sanitizer_cov_trace_pc_guard_init(uint32_t* start, uint32_t* stop) {
  static uint64_t N;  // Counter for the guards.
  if (start == stop || *start) return;  // Initialize only once.
  /* printf("INIT: %p %p\n", start, stop); */
  for (uint32_t *x = start; x < stop; x++)
    *x = ++N;  // Guards should start from 1.
}

void __sanitizer_cov_trace_pc_guard(uint32_t *guard) {
  if (has_shm) {
    shm[*guard % 65536]++;
  }
}
