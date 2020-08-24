#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

uint64_t EnabledHash(const void * key, int len, unsigned int seed);

// Hashing algorithm from Redis.
/* Our hash function is MurmurHash2, 64 bit version.
 * It was modified for Redis in order to provide the same result in
 * big and little endian archs(endian neutral). */
uint64_t MurmurHash64A(const void * key, int len, unsigned int seed);


#ifdef __cplusplus
}
#endif
