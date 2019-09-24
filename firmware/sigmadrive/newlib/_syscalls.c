#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>

int
kill(pid_t pid, int sig);

__attribute__((weak)) int
kill(pid_t pid __attribute__((unused)), int sig __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int
_chown(const char* path, uid_t owner, gid_t group);

int
_close(int fildes);

int
_execve(char* name, char** argv, char** env);

int
_fork(void);

int
_fstat(int fildes, struct stat* st);

int
_getpid(void);

int
_gettimeofday(struct timeval* ptimeval, void* ptimezone);

int
_isatty(int file);

int
_kill(int pid, int sig);

int
_link(char* existing, char* _new);

int
_lseek(int file, int ptr, int dir);

int
_open(char* file, int flags, int mode);

int
_read(int file, char* ptr, int len);

int
_readlink(const char* path, char* buf, size_t bufsize);

int
_stat(const char* file, struct stat* st);

int
_symlink(const char* path1, const char* path2);

clock_t
_times(struct tms* buf);

int
_unlink(char* name);

int
_wait(int* status);

int
_write(int file, char* ptr, int len);

int
_read(int file, char* ptr, int len);


// Definitions

int __attribute__((weak))
_chown(const char* path __attribute__((unused)),
    uid_t owner __attribute__((unused)), gid_t group __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_close(int fildes __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_execve(char* name __attribute__((unused)), char** argv __attribute__((unused)),
    char** env __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_fork(void)
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_fstat(int fildes __attribute__((unused)),
    struct stat* st __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_getpid(void)
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_gettimeofday(struct timeval* ptimeval __attribute__((unused)),
    void* ptimezone __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_isatty(int file __attribute__((unused)))
{
  errno = ENOSYS;
  return 0;
}

int __attribute__((weak))
_kill(int pid __attribute__((unused)), int sig __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_link(char* existing __attribute__((unused)),
    char* _new __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_lseek(int file __attribute__((unused)), int ptr __attribute__((unused)),
    int dir __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_open(char* file __attribute__((unused)), int flags __attribute__((unused)),
    int mode __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_readlink(const char* path __attribute__((unused)),
    char* buf __attribute__((unused)), size_t bufsize __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_stat(const char* file __attribute__((unused)),
    struct stat* st __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_symlink(const char* path1 __attribute__((unused)),
    const char* path2 __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

clock_t __attribute__((weak))
_times(struct tms* buf __attribute__((unused)))
{
  errno = ENOSYS;
  return ((clock_t) -1);
}

int __attribute__((weak))
_unlink(char* name __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_wait(int* status __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

