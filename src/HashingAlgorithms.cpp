#include "HashingAlgorithms.h"
#include "MurmurHash3.h"

uint64_t EnabledHash(const void * key, int len, unsigned int seed)
{
	//return MurmurHash64A(key, len, seed);

	//uint32_t seed = 0x507d12c3;
	uint64_t out[2];
	MurmurHash3_x64_128(key, len, seed, out);

	// Truncate to 64 bit.
	return out[0];
}

uint64_t MurmurHash64A(const void * key, int len, unsigned int seed) {
	const uint64_t m = 0xc6a4a7935bd1e995;
	const int r = 47;
	uint64_t h = seed ^ (len * m);
	const uint8_t *data = (const uint8_t *)key;
	const uint8_t *end = data + (len - (len & 7));

	while (data != end) {
		uint64_t k;

#if (BYTE_ORDER == LITTLE_ENDIAN)
		k = *((uint64_t*)data);
#else
		k = (uint64_t)data[0];
		k |= (uint64_t)data[1] << 8;
		k |= (uint64_t)data[2] << 16;
		k |= (uint64_t)data[3] << 24;
		k |= (uint64_t)data[4] << 32;
		k |= (uint64_t)data[5] << 40;
		k |= (uint64_t)data[6] << 48;
		k |= (uint64_t)data[7] << 56;
#endif

		k *= m;
		k ^= k >> r;
		k *= m;
		h ^= k;
		h *= m;
		data += 8;
	}

	switch (len & 7) {
	case 7: h ^= (uint64_t)data[6] << 48;
	case 6: h ^= (uint64_t)data[5] << 40;
	case 5: h ^= (uint64_t)data[4] << 32;
	case 4: h ^= (uint64_t)data[3] << 24;
	case 3: h ^= (uint64_t)data[2] << 16;
	case 2: h ^= (uint64_t)data[1] << 8;
	case 1: h ^= (uint64_t)data[0];
		h *= m;
	};

	h ^= h >> r;
	h *= m;
	h ^= h >> r;
	return h;
}