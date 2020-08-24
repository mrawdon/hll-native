#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
int getP();

void serializeHLLObject(char* hllObject, char* out, int len);
void deserializeHLLObject(char** hllObject, const char* in, int len);

int pfselftestCommand();

void *createHLLObject(void);

void *createDenseHLLObject(void);

// High level REDIS commands converted to not use redis.

void destroyHLLObject(void* o);
int isHLLObjectOrReply(void *o);
// calculate approximate cardinality, c is an array of HLL objects and num is how many are in the array.
uint64_t pfcountCommand(void *c[], int num);

// Result goes into c[0], it will be created if empty.
// Merges HLL Objects
int pfmergeCommand(void * c[], int num);

// Takes an array of pointers, c[0] is a HLL object the rest are blocks of data length elemlen.
int pfaddCommand(void* c[], int len, int elemlen, uint64_t *hashOut);

// Takes an array with one pointer to a HLL object in it, adds the hash to c[0]
int hllAddHash(void* o[], uint64_t hashIn);

// High level HLL commands used by REDIS commands.

/* Implements the SUM operation for uint8_t data type which is only used
* internally as speedup for PFCOUNT with multiple keys. */
double hllRawSum(uint8_t *registers, double *PE, int *ezp);

/* Return the approximated cardinality of the set based on the harmonic
* mean of the registers values. 'hdr' points to the start of the SDS
* representing the String object holding the HLL representation.
*
* If the sparse representation of the HLL object is not valid, the integer
* pointed by 'invalid' is set to non-zero, otherwise it is left untouched.
*
* hllCount() supports a special internal-only encoding of HLL_RAW, that
* is, hdr->registers will point to an uint8_t array of HLL_REGISTERS element.
* This is useful in order to speedup PFCOUNT when called against multiple
* keys (no need to work with 6-bit integers encoding). */
uint64_t hllCount(struct hllhdr *hdr, int *invalid);

/* Call hllDenseAdd() or hllSparseAdd() according to the HLL encoding. */
int hllAdd(void *o, unsigned char *ele, size_t elesize);

/* Merge by computing MAX(registers[i],hll[i]) the HyperLogLog 'hll'
* with an array of uint8_t HLL_REGISTERS registers pointed by 'max'.
*
* The hll object must be already validated via isHLLObjectOrReply()
* or in some other way.
*
* If the HyperLogLog is sparse and is found to be invalid, RETURN_BAD
* is returned, otherwise the function always succeeds. */
int hllMerge(uint8_t *max, void *hll);



// Low level HLL commands.
int hllSparseToDense(void *o);

/*
int hllDenseAdd(uint8_t *registers, unsigned char *ele, size_t elesize);
double hllDenseSum(uint8_t *registers, double *PE, int *ezp);

int hllSparseAdd(char *o, unsigned char *ele, size_t elesize);
double hllSparseSum(uint8_t *sparse, int sparselen, double *PE, int *ezp, int *invalid);
*/

#ifdef __cplusplus
}
#endif
